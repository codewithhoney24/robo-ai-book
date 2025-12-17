"""
Script to initialize the Qdrant vector store with book content.
This script loads sample book content and indexes it for RAG retrieval.
"""

import asyncio
import sys
import argparse
from pathlib import Path
import json
import logging

# Add the src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from src.services.book_content_service import book_content_service
from src.models.book_content import BookContent
from src.config.logging_config import rag_logger

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

async def load_sample_content():
    """Load sample book content into the vector store."""
    rag_logger.info("Starting to load sample book content...")

    # Create BookContent objects from the sample data
    content_blocks = []
    for block_data in SAMPLE_BOOK_CONTENT:
        content_block = BookContent(
            id=block_data["id"],
            content=block_data["content"],
            source_location=block_data["source_location"],
            book_id=block_data["book_id"]
        )
        content_blocks.append(content_block)

    rag_logger.info(f"Created {len(content_blocks)} content blocks to index")

    # Index the content
    success = book_content_service.index_book_content(
        book_id="ai_robotics_book",
        content_blocks=content_blocks
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

        content_blocks = []
        for block_data in content_data:
            content_block = BookContent(
                id=block_data.get("id", f"content_{len(content_blocks)}"),
                content=block_data.get("content", ""),
                source_location=block_data.get("source_location", ""),
                book_id=block_data.get("book_id", "default_book")
            )
            content_blocks.append(content_block)

        rag_logger.info(f"Loaded {len(content_blocks)} content blocks from file")

        # Index the content
        success = book_content_service.index_book_content(
            book_id=content_data[0].get("book_id", "default_book"),
            content_blocks=content_blocks
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
        success = asyncio.run(load_sample_content())
    elif len(sys.argv) == 1:
        # No arguments - use sample content by default
        success = asyncio.run(load_sample_content())
    else:
        parser.print_help()
        sys.exit(1)

    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()