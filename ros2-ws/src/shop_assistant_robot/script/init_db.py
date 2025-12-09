import pandas as pd
import numpy as np
import kagglehub
import weaviate
import ast
import os
from tqdm import tqdm

print("=" * 70)
print("SHOP ASSISTANT DATABASE INITIALIZATION")
print("=" * 70)

# ============================================================================
# STEP 1: Download Kaggle Data
# ============================================================================
print("\n[1/7] Downloading Kaggle dataset...")
path = kagglehub.dataset_download("thedevastator/grocery-product-prices-for-australian-states")
files = os.listdir(path)
df = pd.read_csv(os.path.join(path, files[0]), index_col='index')
print(f"✓ Downloaded {len(df):,} rows")

# ============================================================================
# STEP 2: Data Cleaning
# ============================================================================
print("\n[2/7] Cleaning data...")
df = df[df['is_estimated'] == 0]
df['in_stock'] = df['in_stock'].fillna(True)
df = df.rename(columns=lambda x: x.strip())

text_cols = ["city", "Postal_code"]
for col in text_cols:
    df[col] = df[col].astype(str).str.strip().str.lower()

print(f"✓ Cleaned to {len(df):,} rows")

# ============================================================================
# STEP 3: Select Best Store
# ============================================================================
print("\n[3/7] Selecting best store...")

grouped = df.groupby(["city", "Postal_code"])
stats = []

for name, g in grouped:
    city, postal = name
    total_rows = len(g)
    unique_skus = g["Sku"].nunique(dropna=True)
    unique_cats = g["Category"].nunique(dropna=True)
    in_stock_mask = g["in_stock"].astype(str).str.lower().isin(["1", "true", "yes", "y", "t"])
    in_stock_ratio = in_stock_mask.sum() / max(1, total_rows)
    key_fields = ["Sku", "Product_Name", "Package_price", "Brand"]
    completeness = g[key_fields].notna().mean(axis=1).mean()
    
    stats.append({
        "city": city,
        "postal_code": postal,
        "total_rows": total_rows,
        "unique_skus": unique_skus,
        "unique_categories": unique_cats,
        "in_stock_ratio": in_stock_ratio,
        "completeness": completeness,
    })

def min_max_series(s):
    if s.max() == s.min():
        return pd.Series([0.5] * len(s), index=s.index)
    return (s - s.min()) / (s.max() - s.min())

stats_df = pd.DataFrame(stats)
stats_df["s_sku"] = min_max_series(stats_df["unique_skus"])
stats_df["s_cat"] = min_max_series(stats_df["unique_categories"])
stats_df["s_stock"] = min_max_series(stats_df["in_stock_ratio"])
stats_df["s_comp"] = min_max_series(stats_df["completeness"])

w_sku, w_cat, w_stock, w_comp = 0.30, 0.25, 0.20, 0.15
stats_df["score"] = (
    stats_df["s_sku"] * w_sku +
    stats_df["s_cat"] * w_cat +
    stats_df["s_stock"] * w_stock +
    stats_df["s_comp"] * w_comp
)

best = stats_df.sort_values("score", ascending=False).iloc[0]
df = df[(df['city'] == best['city']) & (df['Postal_code'] == best['postal_code'])].copy()
df.dropna(inplace=True)
df = df.sort_values(['Category', 'Sub_category']).reset_index(drop=True)

print(f"✓ Selected: {best['city'].title()}, Postal Code: {best['postal_code']}")
print(f"  Products: {len(df):,}")
print(f"  Score: {best['score']:.3f}")
print(f"  Categories: {df['Category'].nunique()}")

# ============================================================================
# STEP 4: Assign to 40 Positions
# ============================================================================
print("\n[4/7] Assigning products to 40 positions...")

def assign_contiguous(df, n_pos):
    N = len(df)
    base = N // n_pos
    r = N % n_pos
    pos_labels = []
    for i in range(n_pos):
        size = base + (1 if i < r else 0)
        pos_labels += [i] * size
    return pos_labels[:N]

df['position'] = assign_contiguous(df, 40)
print(f"✓ Assigned {len(df):,} products to 40 positions")

# ============================================================================
# STEP 5: Load Coordinate Grid
# ============================================================================
print("\n[5/7] Loading 40 coordinate points...")

coord_path = os.path.expanduser("~/MIE1075-project/ros2-ws/src/shop_assistant_robot/resource/db/Datasets_position_with_direction.xlsx")
if not os.path.exists(coord_path):
    print(f"✗ ERROR: Coordinate file not found at {coord_path}")
    exit(1)

df_xy = pd.read_excel(coord_path, sheet_name="Sheet1")
xy_cols = [col for col in df_xy.columns if col.startswith('x')]

def str_to_tuple(s):
    return ast.literal_eval(s)

for col in xy_cols:
    df_xy[col] = df_xy[col].apply(str_to_tuple)

df_long = df_xy.melt(value_vars=xy_cols, var_name='x_col', value_name='xy')
df_long[['x', 'y']] = pd.DataFrame(df_long['xy'].tolist(), index=df_long.index)
df_long = df_long.drop(columns=['xy', 'x_col'])
df_long['position'] = range(len(df_long))

df_final = pd.merge(df, df_long, on='position', how='left')

print(f"✓ Loaded 40 coordinates")
print(f"  X range: ({df_final['x'].min():.2f}, {df_final['x'].max():.2f})")
print(f"  Y range: ({df_final['y'].min():.2f}, {df_final['y'].max():.2f})")

# ============================================================================
# STEP 6: Create Search Text
# ============================================================================
print("\n[6/7] Creating search text...")

df_final['search_text'] = df_final.apply(
    lambda row: f"{row['Product_Name']} {row['Brand']} {row['Category']} {row['Sub_category']} {row['Product_Group']}",
    axis=1
)

print(f"✓ Created search text for {len(df_final):,} products")

# ============================================================================
# STEP 7: Import to Weaviate
# ============================================================================
print("\n[7/7] Importing to Weaviate...")

# Connect
client = weaviate.Client("http://localhost:8080")

if not client.is_ready():
    print("✗ ERROR: Cannot connect to Weaviate at http://localhost:8080")
    print("  Make sure Weaviate is running: docker-compose ps")
    exit(1)

print("✓ Connected to Weaviate")

# Delete existing schema if exists
try:
    client.schema.delete_class("Product")
    print("✓ Deleted existing Product class")
except:
    pass

# Create schema
schema = {
    "class": "Product",
    "description": "Grocery products from Yarragon store",
    "vectorizer": "text2vec-transformers",
    "moduleConfig": {
        "text2vec-transformers": {
            "vectorizeClassName": False
        }
    },
    "properties": [
        {"name": "productName", "dataType": ["text"], "description": "Product name"},
        {"name": "category", "dataType": ["text"], "description": "Main category"},
        {"name": "subCategory", "dataType": ["text"], "description": "Sub category"},
        {"name": "productGroup", "dataType": ["text"], "description": "Product group"},
        {"name": "brand", "dataType": ["text"], "description": "Brand name"},
        {"name": "price", "dataType": ["number"], "description": "Package price"},
        {"name": "pricePerUnit", "dataType": ["text"], "description": "Price per unit"},
        {"name": "packageSize", "dataType": ["text"], "description": "Package size"},
        {"name": "sku", "dataType": ["text"], "description": "SKU"},
        {"name": "inStock", "dataType": ["boolean"], "description": "In stock"},
        {"name": "isSpecial", "dataType": ["boolean"], "description": "Special offer"},
        {"name": "locationX", "dataType": ["number"], "description": "X coordinate"},
        {"name": "locationY", "dataType": ["number"], "description": "Y coordinate"},
        {"name": "position", "dataType": ["int"], "description": "Position ID (0-39)"},
        {"name": "searchText", "dataType": ["text"], "description": "Search text",
         "moduleConfig": {"text2vec-transformers": {"skip": False}}}
    ]
}

client.schema.create_class(schema)
print("✓ Created Product schema")

# Batch import
print(f"\n  Importing {len(df_final):,} products...")
client.batch.configure(batch_size=100)

success_count = 0
error_count = 0

with client.batch as batch:
    for idx, row in tqdm(df_final.iterrows(), total=len(df_final), desc="  Progress"):
        try:
            data_object = {
                "productName": str(row['Product_Name']),
                "category": str(row['Category']),
                "subCategory": str(row['Sub_category']),
                "productGroup": str(row['Product_Group']),
                "brand": str(row['Brand']),
                "price": float(row['Package_price']),
                "pricePerUnit": str(row['Price_per_unit']),
                "packageSize": str(row['package_size']),
                "sku": str(row['Sku']),
                "inStock": bool(row['in_stock']),
                "isSpecial": bool(row['is_special']),
                "locationX": float(row['x']),
                "locationY": float(row['y']),
                "position": int(row['position']),
                "searchText": str(row['search_text'])
            }
            
            batch.add_data_object(
                data_object=data_object,
                class_name="Product"
            )
            success_count += 1
        except Exception as e:
            error_count += 1
            if error_count <= 5:
                print(f"\n  ⚠ Error at row {idx}: {e}")

print(f"✓ Import complete: {success_count:,} success, {error_count} errors")

# ============================================================================
# STEP 8: Verify Import
# ============================================================================
print("\n" + "=" * 70)
print("VERIFICATION")
print("=" * 70)

# Count products
result = client.query.aggregate("Product").with_meta_count().do()
count = result['data']['Aggregate']['Product'][0]['meta']['count']
print(f"\n✓ Total products in Weaviate: {count:,}")

# Test search
print("\n  Testing search for 'chicken'...")
result = (
    client.query
    .get("Product", ["productName", "price", "category", "locationX", "locationY"])
    .with_hybrid(query="chicken", alpha=0.5)
    .with_limit(3)
    .do()
)

if result['data']['Get']['Product']:
    print("  ✓ Search working! Top 3 results:")
    for i, product in enumerate(result['data']['Get']['Product'], 1):
        print(f"    {i}. {product['productName']}")
        print(f"       Price: ${product['price']:.2f}")
        print(f"       Location: ({product['locationX']:.2f}, {product['locationY']:.2f})")
else:
    print("  ✗ No results found")

# ============================================================================
# Summary
# ============================================================================
print("\n" + "=" * 70)
print("✅ DATABASE INITIALIZATION COMPLETE!")
print("=" * 70)
print(f"\nSummary:")
print(f"  • Store: {best['city'].title()}, VIC {best['postal_code']}")
print(f"  • Products: {count:,}")
print(f"  • Categories: {df_final['Category'].nunique()}")
print(f"  • Coordinate points: 40")
print(f"  • Weaviate URL: http://localhost:8080")
print(f"\nNext steps:")
print(f"  1. Run ROS2 nodes")
print(f"  2. Start frontend/backend")
print(f"  3. Test voice search")
print("=" * 70)
