"""
Script to initialize the Qdrant vector store with book content.
This script loads sample book content and indexes it for RAG retrieval.
"""

import sys
import argparse
import json
from pathlib import Path
import logging
import uuid
from datetime import datetime

# Add the src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from src.services.embedding_service import embedding_service
from src.services.vector_store_service import vector_store_service
from src.models.book_content import BookContent
from src.config.logging_config import rag_logger
from src.config.settings import settings


# Sample book content for AI robotics
SAMPLE_BOOK_CONTENT = [
    {
        "id": "ch1_introduction",
        "content": "Chapter 1: Introduction to AI and Robotics. Artificial Intelligence (AI) and robotics are two rapidly evolving fields that have transformed the way we think about technology and its applications. AI refers to the simulation of human intelligence in machines that are programmed to think like humans and mimic their actions. Robotics is the branch of technology that deals with the design, construction, operation, and use of robots. When combined, AI and robotics create systems that can perceive, reason, act, and adapt to their environment. This integration has led to significant advancements in various fields, including manufacturing, healthcare, space exploration, and domestic applications.",
        "source_location": "Chapter 1, Page 1-15",
        "book_id": "ai_robotics_book"
    },
    {
        "id": "ch2_foundations",
        "content": "Chapter 2: Foundations of AI. The foundations of AI are rooted in mathematics, computer science, and cognitive psychology. Key components include machine learning, natural language processing, computer vision, and expert systems. Machine learning, a subset of AI, involves algorithms that improve automatically through experience. It includes supervised learning (with labeled data), unsupervised learning (finding hidden patterns), and reinforcement learning (learning through interaction with an environment). Deep learning, a specialized subset of machine learning, uses neural networks with multiple layers to analyze various factors of data.",
        "source_location": "Chapter 2, Page 16-45",
        "book_id": "ai_robotics_book"
    },
    {
        "id": "ch3_robotics_basics",
        "content": "Chapter 3: Robotics Basics. Robotics involves the design, construction, and operation of robots. The key components of a robot include sensors (for perceiving the environment), actuators (for movement), a controller (the 'brain' of the robot), and a power supply. There are various types of robots including industrial robots, service robots, medical robots, and military robots. The design process involves considering kinematics (motion), dynamics (forces), and control systems. Modern robots often incorporate AI to enhance their capabilities, enabling them to adapt and learn from their environment.",
        "source_location": "Chapter 3, Page 46-78",
        "book_id": "ai_robotics_book"
    },
    {
        "id": "ch4_perception",
        "content": "Chapter 4: Perception in AI and Robotics. Perception is crucial for AI systems and robots to interact with the real world. It involves processing sensory information to understand the environment. In robotics, perception systems typically include cameras, LiDAR, ultrasonic sensors, and other devices to gather information about the surroundings. Computer vision is a key component, enabling systems to interpret and analyze visual information. Simultaneous Localization and Mapping (SLAM) is a technique used by robots to build a map of an unknown environment while simultaneously keeping track of their location within it.",
        "source_location": "Chapter 4, Page 79-110",
        "book_id": "ai_robotics_book"
    },
    {
        "id": "ch5_planning",
        "content": "Chapter 5: Planning and Decision Making. Planning and decision making are essential functions for autonomous agents and robots. AI planning involves determining a sequence of actions to achieve a goal. This can be done in various environments, including deterministic and non-deterministic ones. Motion planning in robotics focuses on finding a path for a robot to move from one position to another while avoiding obstacles. Decision making in AI often involves reasoning under uncertainty, using techniques like probability theory, decision trees, and utility theory. The integration of planning and perception allows robots to operate effectively in dynamic environments.",
        "source_location": "Chapter 5, Page 111-145",
        "book_id": "ai_robotics_book"
    },
    {
        "id": "ch6_learning",
        "content": "Chapter 6: Machine Learning in Robotics. Machine learning enables robots to improve their performance based on experience. Supervised learning can be used to teach robots specific tasks with labeled examples. Unsupervised learning helps robots discover patterns in their sensory data. Reinforcement learning is particularly useful for robotics, as it allows robots to learn behaviors through trial and error, receiving rewards or penalties based on their actions. Deep reinforcement learning combines deep neural networks with reinforcement learning, enabling robots to learn complex behaviors from raw sensory inputs.",
        "source_location": "Chapter 6, Page 146-180",
        "book_id": "ai_robotics_book"
    },
    {
        "id": "ch7_human_robot_interaction",
        "content": "Chapter 7: Human-Robot Interaction (HRI). Human-Robot Interaction is a multidisciplinary field focused on the design, development, and evaluation of robots for human use. Effective HRI requires understanding social cues, communication methods, and human psychology. Robots designed for human interaction must be able to recognize emotions, understand natural language, and respond appropriately to social signals. Trust and acceptance are critical factors in HRI, as users must feel comfortable and safe when interacting with robots. Safety protocols and ethical considerations are paramount in robots that operate in close proximity to humans.",
        "source_location": "Chapter 7, Page 181-210",
        "book_id": "ai_robotics_book"
    },
    {
        "id": "ch8_applications",
        "content": "Chapter 8: Applications of AI and Robotics. AI and robotics have found applications in numerous fields. In manufacturing, industrial robots increase efficiency and precision. In healthcare, robots assist in surgery, rehabilitation, and patient care. Autonomous vehicles use AI for navigation and decision-making. In agriculture, robots are used for harvesting, planting, and monitoring crops. In space exploration, robots enable missions in environments too dangerous for humans. Service robots are becoming common in homes and businesses, performing tasks like cleaning and assistance. The future promises even more sophisticated applications as these technologies continue to advance.",
        "source_location": "Chapter 8, Page 211-240",
        "book_id": "ai_robotics_book"
    }
]

def index_book_content(book_id: str, content_blocks) -> bool:
    """
    Index book content in the vector store for RAG retrieval
    """
    try:
        # Generate embeddings for all content blocks
        texts = [block["content"] for block in content_blocks]
        embeddings = embedding_service.generate_embeddings(texts)

        # Prepare documents for batch insertion
        documents = []
        for i, block in enumerate(content_blocks):
            # Qdrant requires point IDs to be unsigned integers or UUIDs
            # We'll use UUIDs to ensure they are unique
            import uuid
            doc_id = str(uuid.uuid4())

            documents.append({
                "doc_id": doc_id,
                "content": block["content"],
                "embedding": embeddings[i],
                "source_location": block["source_location"],
                "book_id": block["book_id"]
            })

        # Add all documents to the vector store in a batch
        success = vector_store_service.batch_add_documents(documents)

        if success:
            rag_logger.info(f"Successfully indexed {len(content_blocks)} content blocks for book {book_id}")
            return True
        else:
            rag_logger.error(f"Failed to index content blocks for book {book_id}")
            return False

    except Exception as e:
        rag_logger.error(f"Error indexing book content: {str(e)}")
        return False

def load_sample_content():
    """Load sample book content into the vector store."""
    rag_logger.info("Starting to load sample book content...")

    rag_logger.info(f"Created {len(SAMPLE_BOOK_CONTENT)} content blocks to index")

    # Index the content
    success = index_book_content(
        book_id="ai_robotics_book",
        content_blocks=SAMPLE_BOOK_CONTENT
    )

    if success:
        rag_logger.info("Successfully loaded sample book content to vector store")
        print("Sample book content has been successfully loaded to the vector store!")
    else:
        rag_logger.error("Failed to load sample book content")
        print("Error: Failed to load sample book content to the vector store")

    return success

def load_content_from_file(file_path: str):
    """Load book content from a JSON file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content_data = json.load(f)

        if not isinstance(content_data, list):
            raise ValueError("JSON file must contain an array of content blocks")

        # Validate that all required fields exist
        for i, item in enumerate(content_data):
            if not all(key in item for key in ["id", "content", "source_location", "book_id"]):
                raise ValueError(f"Item at index {i} is missing required fields")

        rag_logger.info(f"Loaded {len(content_data)} content blocks from file")

        # Index the content
        success = index_book_content(
            book_id=content_data[0]["book_id"],
            content_blocks=content_data
        )

        if success:
            rag_logger.info(f"Successfully loaded book content from {file_path}")
            print(f"Book content from {file_path} has been successfully loaded to the vector store!")
        else:
            rag_logger.error(f"Failed to load book content from {file_path}")
            print(f"Error: Failed to load book content from {file_path}")

        return success
    except Exception as e:
        rag_logger.error(f"Error loading content from file: {str(e)}")
        print(f"Error loading content from file: {str(e)}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Initialize the book content vector store')
    parser.add_argument('--book-content-path', type=str, help='Path to JSON file containing book content')
    parser.add_argument('--sample', action='store_true', help='Use sample book content')

    args = parser.parse_args()

    if args.book_content_path:
        # Load from specified file
        success = load_content_from_file(args.book_content_path)
    elif args.sample:
        # Load sample content
        success = load_sample_content()
    elif len(sys.argv) == 1:
        # No arguments - use sample content by default
        success = load_sample_content()
    else:
        parser.print_help()
        sys.exit(1)

    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()