import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import weaviate
import json
import spacy
import os
from dotenv import load_dotenv

load_dotenv()
WEAVIATE_HOST = os.getenv("WEAVIATE_HOST", "localhost")
WEAVIATE_PORT = os.getenv("WEAVIATE_PORT", "8080")
WEAVIATE_GRPC_PORT = os.getenv("WEAVIATE_GRPC_PORT", "50051")
WEAVIATE_SCHEME = os.getenv("WEAVIATE_SCHEME", "http")


class WeaviateSearchNode(Node):
    def __init__(self):
        super().__init__('weaviate_search_node')
        # load spaCy NLP model
        self.get_logger().info('loading spaCy model...')
        self.nlp = spacy.load("en_core_web_sm")
        self.get_logger().info('spaCy loaded')

        # Connect to Weaviate
        self.get_logger().info('Connecting to Weaviate...')
        try:
            self.client = weaviate.Client(
                f"{WEAVIATE_SCHEME}://{WEAVIATE_HOST}:{WEAVIATE_PORT}",
            )
            # self.client = weaviate.Client("http://localhost:8080")

            if not self.client.is_ready():
                self.get_logger().error('Weaviate not ready!')
                raise Exception('Weaviate connection failed')

            self.get_logger().info('Connected to Weaviate')

        except Exception as e:
            self.get_logger().error(f'Failed to connect to Weaviate: {e}')
            raise

        # Publishers
        self.results_pub = self.create_publisher(
            String,
            '/search/results',
            10
        )

        self.location_pub = self.create_publisher(
            PoseStamped,
            '/voice_command/target_coords',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/search/status',
            10
        )

        # Subscribers
        self.query_sub = self.create_subscription(
            String,
            '/search/query',
            self.search_callback,
            10
        )

        self.get_logger().info('Weaviate Search Node Ready')
        self.get_logger().info('  Subscribes: /search/query')
        self.get_logger().info('  Publishes: /search/results, /navigation/target, /search/status')

    def search_callback(self, msg):
        """Handle search query"""
        query = msg.data.strip()

        if not query:
            return

        self.get_logger().info(f'Search query: "{query}"')
        keyword = self.extract_product_keywords(query)
        self.publish_status(f'Searching for: {keyword}')

        # Perform search
        results = self.hybrid_search(keyword)

        if results:
            self.get_logger().info(f'Found {len(results)} results')
            self.publish_results(results, query)
            self.send_to_navigation(results[0])
        else:
            self.get_logger().warn(f'No results for: {query}')
            self.publish_status(f'No results found for: {query}')

    def hybrid_search(self, query, limit=3):
        """
        Perform hybrid search (vector + keyword)
        """
        try:
            result = (
                self.client.query
                .get("Product", [
                    "productName", "category", "subCategory",
                    "brand", "price", "sku",
                    "locationX", "locationY", "position",
                    "inStock", "isSpecial"
                ])
                .with_hybrid(
                    query=query,
                    alpha=0.5  # 0.5 = balanced vector + keyword search
                )
                .with_where({
                    "path": ["inStock"],
                    "operator": "Equal",
                    "valueBoolean": True
                })
                .with_limit(limit)
                .do()
            )

            products = result['data']['Get']['Product']
            return products

        except Exception as e:
            self.get_logger().error(f'Search error: {e}')
            return []

    def publish_results(self, results, query):
        """Publish search results as JSON"""
        msg = String()
        msg.data = json.dumps({
            'query': query,
            'count': len(results),
            'products': results
        }, ensure_ascii=False)

        self.results_pub.publish(msg)

        # Status message
        product_names = [r['productName'] for r in results[:3]]
        status = f"Found: {', '.join(product_names)}"
        if len(results) > 3:
            status += f" (and {len(results) - 3} more)"

        self.publish_status(status)

    def send_to_navigation(self, product):
        """Send navigation target to navigation system"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position.x = float(product['locationX'])
        pose.pose.position.y = float(product['locationY'])
        pose.pose.position.z = 0.0

        # Set orientation (facing forward)
        pose.pose.orientation.w = 1.0

        self.location_pub.publish(pose)

        self.get_logger().info(
            f'Navigation target: {product["productName"]} '
            f'at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

    def publish_status(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def extract_product_keywords(self, user_query):
        """extract product keywords using spaCy"""
        doc = self.nlp(user_query)

        keywords = []
        for token in doc:
            if token.pos_ in ['NOUN', 'PROPN'] and not token.is_stop:
                keywords.append(token.text.lower())
            elif token.pos_ == 'ADJ':
                keywords.append(token.text.lower())

        keyword_string = ' '.join(keywords)
        self.get_logger().info(
            f'"{user_query}" → keywords: "{keyword_string}"')

        return keyword_string if keyword_string else user_query.lower()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = WeaviateSearchNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()
