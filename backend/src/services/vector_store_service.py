from typing import List, Optional, Dict, Any
from qdrant_client.http import models
from qdrant_client.http.exceptions import ResponseHandlingException
from pydantic import ValidationError
from ..config.database_config import db_config
from ..config.logging_config import db_logger
from ..models.retrieved_context import RetrievedContext
from ..config.settings import settings


class VectorStoreService:
    """
    Service for interacting with Qdrant vector database
    """

    def __init__(self):
        self.qdrant_client = db_config.get_qdrant_client()
        self.collection_name = settings.qdrant_collection_name

        # Log the status of the Qdrant client
        if self.qdrant_client is None:
            db_logger.warning("Qdrant client is not available - vector store functionality will be disabled")
        else:
            db_logger.info("Qdrant client initialized successfully")

    def search(
        self,
        query_embedding: List[float],
        query_text: str,
        limit: int = 5,
        filters: Optional[Dict[str, Any]] = None,
        selected_text: Optional[str] = None
    ) -> List[RetrievedContext]:
        """
        Search for relevant documents based on the query embedding
        If selected_text is provided, prioritize contexts that are similar to the selected text
        """
        # Check if Qdrant client is available
        if self.qdrant_client is None:
            db_logger.warning("Qdrant client not available, returning empty results")
            return []

        try:
            # Build search filters if provided
            search_filter = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    ))
                search_filter = models.Filter(must=filter_conditions)

            # Perform the primary search in Qdrant using the newer client API
            # which exposes `query_points` for vector queries
            query_response = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                query_filter=search_filter,
                limit=limit,
                with_payload=True,
            )

            # Convert search results to RetrievedContext objects
            retrieved_contexts = []
            for result in query_response.points:
                # Extract book_id from the payload for use in RetrievedContext
                payload = result.payload or {}
                context = RetrievedContext(
                    id=result.id,
                    content=payload.get("content", ""),
                    relevance_score=result.score,
                    source_location=payload.get("source_location", ""),
                    query_id=query_text  # Using the query text as the query ID for this context
                )
                retrieved_contexts.append(context)

            # If selected_text is provided, prioritize or add contexts that are similar to the selected text
            if selected_text:
                retrieved_contexts = self._prioritize_contexts_for_selected_text(
                    retrieved_contexts, selected_text, query_text
                )

            db_logger.info(f"Retrieved {len(retrieved_contexts)} contexts for query: {query_text[:50]}...")
            return retrieved_contexts

        except Exception as e:
            db_logger.error(f"Error during vector search: {str(e)}")
            # Handle dimension mismatch errors from Qdrant by attempting to recreate collection
            err_str = str(e)
            if "Vector dimension error" in err_str or "expected dim" in err_str:
                try:
                    # Parse expected dimension if available
                    import re
                    m = re.search(r"expected dim:\s*(\d+), got\s*(\d+)", err_str)
                    expected = int(m.group(1)) if m else None
                    got = int(m.group(2)) if m else len(query_embedding)
                except Exception:
                    expected = None
                    got = len(query_embedding)

                # Attempt to fix by recreating collection with correct size
                try:
                    db_logger.info("Attempting to recreate Qdrant collection with correct vector size: %s", got)
                    self.qdrant_client.delete_collection(self.collection_name)
                    self.qdrant_client.create_collection(
                        collection_name=self.collection_name,
                        vectors_config=models.VectorParams(size=got, distance=models.Distance.COSINE),
                    )
                    db_logger.info("Recreated collection %s with vector size %s", self.collection_name, got)
                    # Retry search once
                    query_response = self.qdrant_client.query_points(
                        collection_name=self.collection_name,
                        query=query_embedding,
                        query_filter=search_filter,
                        limit=limit,
                        with_payload=True,
                    )
                    retrieved_contexts = []
                    for result in query_response.points:
                        # Extract book_id from the payload for use in RetrievedContext
                        payload = result.payload or {}
                        context = RetrievedContext(
                            id=result.id,
                            content=payload.get("content", ""),
                            relevance_score=result.score,
                            source_location=payload.get("source_location", ""),
                            query_id=query_text
                        )
                        retrieved_contexts.append(context)

                    db_logger.info(f"Retrieved {len(retrieved_contexts)} contexts for query: {query_text[:50]}...")
                    return retrieved_contexts
                except Exception as inner_e:
                    db_logger.error(f"Failed to recreate or search after recreation: {inner_e}")
                    raise e

            raise e

    def _prioritize_contexts_for_selected_text(
        self,
        contexts: List[RetrievedContext],
        selected_text: str,
        original_query: str
    ) -> List[RetrievedContext]:
        """
        Prioritize contexts that are similar to the selected text
        """
        # For now, we'll use a simple approach of re-ranking based on similarity
        # to the selected text. In a more sophisticated implementation, we could
        # do another search specifically for content similar to the selected text.

        from ..services.embedding_service import embedding_service

        try:
            # Generate an embedding for the selected text
            selected_text_embedding = embedding_service.generate_embedding(selected_text)

            # Re-rank contexts based on their similarity to the selected text
            for context in contexts:
                content_embedding = embedding_service.generate_embedding(context.content)
                similarity = embedding_service.calculate_similarity(
                    selected_text_embedding, content_embedding
                )

                # Combine the original relevance score with the similarity to selected text
                # This balances the original relevance with the relevance to the selected text
                combined_score = self._calculate_combined_score(
                    original_score=context.relevance_score,
                    similarity_score=similarity
                )

                context.relevance_score = combined_score

            # Sort the contexts by relevance score in descending order
            contexts.sort(key=lambda x: x.relevance_score, reverse=True)

            return contexts
        except Exception as e:
            db_logger.warning(f"Error prioritizing contexts for selected text: {str(e)}")
            # If prioritization fails, return the original list
            return contexts

    def _calculate_combined_score(self, original_score: float, similarity_score: float) -> float:
        """
        Calculate a combined relevance score that takes into account both the original relevance
        and the similarity to the selected text.

        This implementation uses a weighted average approach, but could be adjusted based on
        performance requirements and desired behavior.
        """
        # Use a weighted average to balance the original relevance score and the similarity to selected text
        # This gives 60% weight to the original relevance and 40% weight to the selected text similarity
        combined_score = (0.6 * original_score) + (0.4 * similarity_score)

        # Ensure the score is between 0 and 1
        return max(0.0, min(1.0, combined_score))

    def add_document(
        self,
        doc_id: str,
        content: str,
        embedding: List[float],
        source_location: str,
        book_id: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Add a document to the vector store
        """
        try:
            # Prepare payload
            payload = {
                "content": content,
                "source_location": source_location,
                "book_id": book_id
            }

            # Add any additional metadata
            if metadata:
                payload.update(metadata)

            # Upsert the document in Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=doc_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )

            db_logger.info(f"Added document {doc_id} to vector store")
            return True

        except Exception as e:
            db_logger.error(f"Error adding document to vector store: {str(e)}")
            return False

    def delete_document(self, doc_id: str) -> bool:
        """
        Delete a document from the vector store
        """
        try:
            self.qdrant_client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[doc_id])
            )

            db_logger.info(f"Deleted document {doc_id} from vector store")
            return True

        except Exception as e:
            db_logger.error(f"Error deleting document from vector store: {str(e)}")
            return False

    def batch_add_documents(
        self,
        documents: List[Dict[str, Any]]
    ) -> bool:
        """
        Add multiple documents to the vector store in a batch
        """
        try:
            points = []
            for doc in documents:
                points.append(
                    models.PointStruct(
                        id=doc["doc_id"],
                        vector=doc["embedding"],
                        payload={
                            "content": doc["content"],
                            "source_location": doc["source_location"],
                            "book_id": doc["book_id"],
                            **(doc.get("metadata", {}))
                        }
                    )
                )

            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            db_logger.info(f"Added {len(documents)} documents to vector store in batch")
            return True

        except Exception as e:
            db_logger.error(f"Error during batch add to vector store: {str(e)}")
            return False


# Global instance
vector_store_service = VectorStoreService()