import cohere
import openai
from typing import List
import numpy as np
from ..config.settings import settings
from ..config.logging_config import embedding_logger
from .caching_service import caching_service


class EmbeddingService:
    """
    Service for generating embeddings using Cohere API with OpenAI fallback
    """

    def __init__(self):
        # Initialize Cohere client if API key is available (primary service)
        if settings.cohere_api_key:
            self.client = cohere.Client(settings.cohere_api_key, timeout=30)
            self.embedding_model = "embed-english-v3.0"  # Using Cohere's English embedding model
            self.use_cohere = True
        elif settings.OPENAI_API_KEY:
            # Configure OpenAI client as fallback
            openai.api_key = settings.OPENAI_API_KEY
            self.embedding_model = "text-embedding-ada-002"  # Using OpenAI's embedding model
            self.use_cohere = False
        else:
            raise ValueError("No valid API key found for embeddings. Please set either COHERE_API_KEY or OPENAI_API_KEY in your environment variables.")

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        try:
            # Try to get from cache first
            cache_key = f"embedding_{hash(text) % 1000000}"  # Use a simple hash as cache key
            cached_result = caching_service.get(cache_key)
            if cached_result is not None:
                embedding_logger.debug(f"Retrieved embedding from cache for text of length {len(text)}")
                return cached_result

            if self.use_cohere:
                response = self.client.embed(
                    texts=[text],
                    model=self.embedding_model,
                    input_type="search_query"
                )
                embedding_result = response.embeddings[0]
            else:
                # Use OpenAI API
                response = openai.embeddings.create(
                    input=[text],
                    model=self.embedding_model
                )
                embedding_result = response.data[0].embedding

            # Cache the result with a default TTL of 1 hour
            caching_service.set(cache_key, embedding_result, ttl_seconds=3600)

            embedding_logger.info(f"Generated embedding for text of length {len(text)}")
            return embedding_result

        except Exception as e:
            embedding_logger.error(f"Error generating embedding: {str(e)}")
            raise e

    def generate_embedding_for_combined_query(self, query: str, selected_text: str = None) -> List[float]:
        """
        Generate embedding for a query combined with selected text if provided
        """
        try:
            if selected_text:
                # Combine the query and selected text for the embedding
                combined_text = f"{query} {selected_text}"
                cache_key = f"combined_embedding_{hash(combined_text) % 1000000}"
            else:
                combined_text = query
                cache_key = f"query_embedding_{hash(combined_text) % 1000000}"

            # Try to get from cache first
            cached_result = caching_service.get(cache_key)
            if cached_result is not None:
                embedding_logger.debug(f"Retrieved combined embedding from cache for text of length {len(combined_text)}")
                return cached_result

            if self.use_cohere:
                response = self.client.embed(
                    texts=[combined_text],
                    model=self.embedding_model,
                    input_type="search_query"
                )
                embedding_result = response.embeddings[0]
            else:
                # Use OpenAI API
                response = openai.embeddings.create(
                    input=[combined_text],
                    model=self.embedding_model
                )
                embedding_result = response.data[0].embedding

            # Cache the result with a default TTL of 1 hour
            caching_service.set(cache_key, embedding_result, ttl_seconds=3600)

            embedding_logger.info(f"Generated embedding for combined query with selected text")
            return embedding_result

        except Exception as e:
            embedding_logger.error(f"Error generating embedding for combined query: {str(e)}")
            raise e

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        """
        try:
            if self.use_cohere:
                response = self.client.embed(
                    texts=texts,
                    model=self.embedding_model,
                    input_type="search_document"  # Using document type for longer passages like book content
                )
                embeddings = response.embeddings
            else:
                # Use OpenAI API
                response = openai.embeddings.create(
                    input=texts,
                    model=self.embedding_model
                )
                embeddings = [item.embedding for item in response.data]

            embedding_logger.info(f"Generated {len(embeddings)} embeddings")
            return embeddings

        except Exception as e:
            embedding_logger.error(f"Error generating embeddings: {str(e)}")
            raise e

    def calculate_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings
        """
        # Convert to numpy arrays
        arr1 = np.array(embedding1)
        arr2 = np.array(embedding2)

        # Calculate cosine similarity
        dot_product = np.dot(arr1, arr2)
        norm1 = np.linalg.norm(arr1)
        norm2 = np.linalg.norm(arr2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        similarity = dot_product / (norm1 * norm2)

        # Ensure the result is between 0 and 1
        return max(0.0, min(1.0, float(similarity)))


# Global instance
embedding_service = EmbeddingService()
