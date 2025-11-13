import sqlite3
import os
import csv

DB_PATH = "src/shop_assistant_robot/resource/db/products.db"


def init_db(product_path, regions_path):
    
    os.makedirs("resource/db", exist_ok=True)

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Todo: change the schema for products
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS products (
            id INTEGER PRIMARY KEY,
            name TEXT,
            category TEXT,
            region TEXT
        );
    """)

    # Todo:change the schema for regions
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS regions (
            region TEXT PRIMARY KEY,
            pos_x REAL,
            pos_y REAL
        );
    """)

    with open(product_path, newline='', encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cursor.execute("""
                INSERT INTO products (id, name, category, region)
                VALUES (?, ?, ?, ?)
            """, (row["id"], row["name"], row["category"], row["region"]))

    with open(regions_path, newline='', encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cursor.execute("""
                INSERT INTO regions (region, pos_x, pos_y)
                VALUES (?, ?, ?)
            """, (row["region"], row["pos_x"], row["pos_y"]))

    
    conn.commit()
    conn.close()

    print("Database initialized:", DB_PATH)


if __name__ == "__main__":
    product_path = 'src/shop_assistant_robot/resource/db/product.csv' # change to the real product csv
    regions_path = 'src/shop_assistant_robot/resource/db/region.csv' # change to the real region csv
    init_db(product_path, regions_path)
