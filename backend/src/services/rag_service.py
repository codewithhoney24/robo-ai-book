import asyncio
import logging
import uuid
from typing import List, Dict, Any, Optional, Union
from pydantic import BaseModel
from datetime import datetime

from src.services.embedding_service import embedding_service
from src.services.vector_store_service import vector_store_service
from src.services.database_service import database_service
from src.services.error_handler import error_handler_service
from src.core.exceptions import DatabaseConnectionError
from src.models.generated_response import GeneratedResponse
from src.models.retrieved_context import RetrievedContext
from src.exceptions import (
    VectorSearchException,
    DatabaseConnectionException,
    EmbeddingGenerationException,
)


logger = logging.getLogger(__name__)


class RagQueryResult(BaseModel):
    """
    Model for RAG query results.
    """
    response: str
    source_documents: List[str]
    confidence_score: float
    metadata: Dict[str, Any] = {}


class RagService:
    """
    Service to handle RAG (Retrieval Augmented Generation) operations.
    This integrates with the database connection management for textbook content retrieval.
    """

    def __init__(self):
        self.default_chunk_size = 500  # Characters
        self.default_top_k = 5  # Number of documents to retrieve

    def process_query_with_rag(self, query: Union[str, "UserQuery"], top_k: Optional[int] = None, book_id: Optional[str] = None) -> GeneratedResponse:
        """
        Synchronous method to process a query using RAG approach.
        This integrates the embedding, vector store, and database services.

        Args:
            query: The user's query
            top_k: Number of top results to return (optional, defaults to default_top_k)
            book_id: Optional book ID to filter results by (optional)

        Returns:
            RagQueryResult containing the response and source documents
        """
        if top_k is None:
            top_k = self.default_top_k

        try:
            # Normalize input to text and UserQuery-like object
            if isinstance(query, str):
                query_text = query
                user_query_obj = None
            else:
                # import here to avoid circular import at module load
                from src.models.user_query import UserQuery
                user_query_obj = query
                query_text = getattr(user_query_obj, "content", "")

            # Validate input
            if not query_text or len(query_text.strip()) == 0:
                # Check if we have a selected_text to use instead
                if user_query_obj and user_query_obj.selected_text:
                    query_text = f"Explain this text: {user_query_obj.selected_text}"
                else:
                    raise ValueError("Query cannot be empty")

            if len(query_text) > 2000:  # Max 2000 characters
                raise ValueError("Query exceeds maximum allowed length")

            # Check if the query is a simple greeting (to avoid unnecessary RAG processing)
            from src.config.settings import settings
            if self._is_greeting_query(query_text, settings):
                response_text = self._generate_greeting_response(query_text)

                # Create GeneratedResponse for greeting
                gen_resp = GeneratedResponse(
                    id=f"resp_{uuid.uuid4().hex}",
                    content=response_text,
                    timestamp=datetime.utcnow(),
                    query_id=(user_query_obj.id if user_query_obj is not None else f"query_{uuid.uuid4().hex}"),
                    relevance_score=1.0,  # Greetings are 100% relevant
                    retrieved_contexts=[]
                )

                # Save the query in the database if we have a UserQuery
                try:
                    if user_query_obj is not None:
                        database_service.create_user_query(user_query_obj)
                    else:
                        # create a simple record for the query text
                        from src.models.user_query import UserQuery
                        database_service.create_user_query(UserQuery(id=f"query_{uuid.uuid4().hex}", content=query_text, user_session_id="default_session"))
                    logger.info("Saved user query to database")
                except Exception as db_error:
                    logger.error(f"Failed to save user query to database: {db_error}")
                    # Don't fail the entire operation if database save fails

                return gen_resp

            # Generate embedding for the query
            embedding = embedding_service.generate_embedding(query_text)
            logger.info(f"Generated embedding for query: {query_text[:50]}...")

            # Search in vector store using the embedding
            # If book_id is provided, pass it as a filter
            filters = {"book_id": book_id} if book_id else None
            retrieved_contexts = vector_store_service.search(
                query_embedding=embedding,
                query_text=query_text,
                limit=top_k,
                filters=filters
            )
            logger.info(f"Retrieved {len(retrieved_contexts)} contexts from vector store")

            # Extract content and source locations from retrieved contexts
            source_documents = []
            context_contents = []
            mapped_contexts = []
            for ctx in retrieved_contexts:
                source_documents.append(ctx.source_location)
                context_contents.append(ctx.content)
                mapped_contexts.append(RetrievedContext(
                    id=ctx.id,
                    content=ctx.content,
                    relevance_score=ctx.relevance_score,
                    source_location=ctx.source_location,
                    query_id=(user_query_obj.id if user_query_obj is not None else query_text)
                ))

            # Generate response based on retrieved contexts
            response_text = self._generate_response_from_contexts(query_text, context_contents)
            logger.info(f"Generated response for query: {query_text[:50]}...")

            # Create GeneratedResponse
            gen_resp = GeneratedResponse(
                id=f"resp_{uuid.uuid4().hex}",
                content=response_text,
                timestamp=datetime.utcnow(),
                query_id=(user_query_obj.id if user_query_obj is not None else f"query_{uuid.uuid4().hex}"),
                relevance_score=max([c.relevance_score for c in mapped_contexts]) if mapped_contexts else 0.0,
                retrieved_contexts=mapped_contexts
            )

            # Save the query in the database if we have a UserQuery
            try:
                if user_query_obj is not None:
                    database_service.create_user_query(user_query_obj)
                else:
                    # create a simple record for the query text
                    from src.models.user_query import UserQuery
                    database_service.create_user_query(UserQuery(id=f"query_{uuid.uuid4().hex}", content=query_text, user_session_id="default_session"))
                logger.info("Saved user query to database")
            except Exception as db_error:
                logger.error(f"Failed to save user query to database: {db_error}")
                # Don't fail the entire operation if database save fails

            return gen_resp

        except ValueError as ve:
            logger.error(f"Input validation error in process_query_with_rag: {ve}")
            raise EmbeddingGenerationException(f"Invalid query input: {str(ve)}")
        except Exception as e:
            logger.error(f"Error in process_query_with_rag: {e}")
            # Wrap other errors in appropriate domain exceptions
            if "embedding" in str(e).lower():
                raise EmbeddingGenerationException(f"Error generating embedding: {str(e)}")
            elif "vector" in str(e).lower() or "search" in str(e).lower():
                raise VectorSearchException(f"Error during vector search: {str(e)}")
            elif "database" in str(e).lower() or "connection" in str(e).lower():
                raise DatabaseConnectionException(f"Database error during RAG processing: {str(e)}")
            else:
                raise VectorSearchException(f"Unexpected error during RAG processing: {str(e)}")

    def _is_greeting_query(self, query_text: str, settings) -> bool:
        """
        Check if the query is a simple greeting based on the settings patterns.

        Args:
            query_text: The query text to check
            settings: The application settings object

        Returns:
            True if query is a greeting, False otherwise
        """
        # Check if the query is short enough to be a greeting
        if len(query_text.strip()) > settings.max_greeting_query_length:
            return False

        # Check if the query matches any of the greeting patterns
        query_lower = query_text.lower().strip()
        for pattern in settings.greeting_patterns:
            if query_lower == pattern or query_lower.startswith(pattern) or query_lower.endswith(pattern):
                return True

        return False

    def _generate_greeting_response(self, query_text: str) -> str:
        """
        Generate a simple greeting response for greeting queries.

        Args:
            query_text: The greeting query text

        Returns:
            A simple greeting response
        """
        query_lower = query_text.lower().strip()

        # Return a greeting that matches the user's greeting style as best as possible
        if query_lower in ['hello', 'hi', 'hey']:
            return "Hello! How can I help you with the AI and Robotics textbook today?"
        elif query_lower in ['greetings']:
            return "Greetings! How can I assist you with your questions about AI and Robotics?"
        elif query_lower in ['good morning']:
            return "Good morning! How can I help you with the AI and Robotics textbook today?"
        elif query_lower in ['good afternoon']:
            return "Good afternoon! How can I assist you with your questions about AI and Robotics?"
        elif query_lower in ['good evening']:
            return "Good evening! How can I help you with the AI and Robotics textbook today?"
        elif query_lower in ['good night']:
            return "Good night! Feel free to ask any questions about AI and Robotics."
        else:
            # If it's some other greeting variation, respond with a standard greeting
            return "Hello! How can I assist you with your questions about AI and Robotics today?"

    def _generate_response_from_contexts(self, query: str, context_contents: List[str]) -> str:
        """
        Generate a response based on the query and retrieved context contents.

        Args:
            query: The original query
            context_contents: List of relevant content strings from the vector store

        Returns:
            Generated response string
        """
        if not context_contents:
            return (
                "I couldn't find any relevant content in the textbook for your query: "
                f"'{query[:50]}...'. Please try rephrasing your question."
            )

        # Combine context contents for response generation
        combined_context = "\n\n".join(context_contents[:3])  # Limit to first 3 contexts

        return (
            f"Based on the textbook content, here's what I found for your query "
            f"'{query[:50]}...':\n\n"
            f"{combined_context}\n\n"
            f"This information was retrieved from the relevant sections of the textbook. "
            f"For more detailed information, please refer to the specific sections mentioned."
        )

    async def query_textbook_content(self, query: str, top_k: Optional[int] = None) -> RagQueryResult:
        """
        Query the textbook content using RAG approach.

        Args:
            query: The user's query
            top_k: Number of top results to return (optional, defaults to default_top_k)

        Returns:
            RagQueryResult containing the response and source documents
        """
        if top_k is None:
            top_k = self.default_top_k

        try:
            # In a real implementation, this would query a vector database
            # to retrieve relevant textbook content based on the query
            # For this simulation, we'll return mock content

            # Simulate retrieval of relevant documents
            source_docs = await self._retrieve_relevant_documents(query, top_k, None)

            # Generate response based on retrieved documents
            response = await self._generate_response(query, source_docs)

            return RagQueryResult(
                response=response,
                source_documents=source_docs,
                confidence_score=0.85,  # Mock confidence score
                metadata={"query": query, "retrieved_docs_count": len(source_docs)}
            )

        except Exception as e:
            logger.error(f"Error in RAG query: {e}")

            # Use error handler to provide fallback
            error_response = await error_handler_service.handle_database_error(
                e,
                fallback_type="service_unavailable",
                context="RAG query"
            )

            return RagQueryResult(
                response=error_response.get("fallback_response",
                            "I'm unable to access the textbook content at the moment. Please try again later."),
                source_documents=[],
                confidence_score=0.0,
                metadata={"error": str(e)}
            )

    async def _retrieve_relevant_documents(self, query: str, top_k: int, session) -> List[str]:
        """
        Retrieve relevant documents from the textbook database.

        Args:
            query: The search query
            top_k: Number of documents to retrieve
            session: Database session

        Returns:
            List of document IDs or paths
        """
        # This is a mock implementation
        # In a real system, this would:
        # 1. Embed the query using a model like sentence-transformers
        # 2. Query a vector database (e.g., Qdrant, Pinecone, Weaviate) for similar embeddings
        # 3. Retrieve the top_k most similar documents

        # Simulate some delay for database operation
        await asyncio.sleep(0.02)

        # Return mock documents based on the query
        mock_docs = [
            f"chapter_{i}_introduction_to_ai.txt"
            for i in range(1, min(top_k + 1, 4))
        ]

        return mock_docs

    async def _generate_response(self, query: str, source_documents: List[str]) -> str:
        """
        Generate a response based on the query and retrieved documents.

        Args:
            query: The original query
            source_documents: List of relevant documents

        Returns:
            Generated response string
        """
        # This is a mock implementation
        # In a real system, this would send the query and documents to an LLM
        # like GPT, Claude, or an open-source model

        # Simulate processing time
        await asyncio.sleep(0.05)

        if not source_documents:
            return (
                "I couldn't find any relevant content in the textbook for your query: "
                f"'{query[:50]}...'. Please try rephrasing your question."
            )

        return (
            f"Based on the textbook content, here's what I found for your query "
            f"'{query[:50]}...':\n\n"
            f"The relevant information is covered in the following chapters: "
            f"{', '.join(source_documents)}.\n\n"
            f"In a real implementation, this would contain a synthesized response "
            f"generated by an LLM based on the content from these chapters."
        )

    async def validate_textbook_access(self) -> bool:
        """
        Validate that the RAG service can access the textbook content database.

        Returns:
            True if access is available, False otherwise
        """
        try:
            # Try to retrieve a simple document or check database connectivity
            # In a real implementation, this might query for a known document
            # For now, we'll just simulate a successful check
            await asyncio.sleep(0.01)  # Simulate minimal database operation
            return True
        except Exception as e:
            logger.error(f"Textbook access validation failed: {e}")
            return False

    async def get_textbook_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the textbook content.

        Returns:
            Dictionary with textbook statistics
        """
        try:
            # In a real implementation, this would query the database for stats
            # such as number of documents, total content size, etc.
            # For now, we'll return mock stats
            await asyncio.sleep(0.02)

            return {
                "total_documents": 150,  # Mock value
                "total_chapters": 12,   # Mock value
                "indexed_content_size_mb": 25.5,  # Mock value
                "last_indexed": "2025-12-14T10:00:00Z",  # Mock value
                "index_health": "good"
            }
        except Exception as e:
            logger.error(f"Failed to get textbook stats: {e}")
            return {"error": str(e)}


# Global instance
rag_service = RagService()