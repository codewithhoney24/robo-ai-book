from typing import List
from ..models.book_content import BookContent
from ..services.embedding_service import embedding_service
from ..services.vector_store_service import vector_store_service
from ..config.logging_config import rag_logger
from ..config.settings import settings


class BookContentService:
    """
    Service for managing book content in the RAG system
    """
    
    def __init__(self):
        pass
    
    def index_book_content(self, book_id: str, content_blocks: List[BookContent]) -> bool:
        """
        Index book content in the vector store for RAG retrieval
        """
        try:
            # Generate embeddings for all content blocks
            texts = [block.content for block in content_blocks]
            embeddings = embedding_service.generate_embeddings(texts)

            # Prepare documents for batch insertion
            documents = []
            for i, block in enumerate(content_blocks):
                # Ensure doc_id is a valid format for Qdrant (UUID or integer)
                # If the original block.id is not a UUID or integer, we'll create a mapping
                # between the original ID and a new UUID for Qdrant
                import uuid
                qdrant_doc_id = str(uuid.uuid4())  # Generate a valid UUID for Qdrant

                documents.append({
                    "doc_id": qdrant_doc_id,
                    "content": block.content,
                    "embedding": embeddings[i],
                    "source_location": block.source_location,
                    "book_id": book_id,
                    "metadata": {
                        "original_id": block.id  # Store the original ID in metadata
                    }
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
    
    def index_text_chunks(self, book_id: str, full_text: str, chunk_size: int = 1000) -> bool:
        """
        Split book text into chunks and index them in the vector store
        """
        try:
            # Split the text into chunks
            chunks = self._split_text_into_chunks(full_text, chunk_size)
            
            # Create BookContent objects
            content_blocks = []
            for i, chunk in enumerate(chunks):
                content_block = BookContent(
                    id=f"{book_id}_chunk_{i}",
                    content=chunk,
                    vector_embedding=[],  # Will be populated during indexing
                    source_location=f"Book {book_id}, Chunk {i}",
                    book_id=book_id
                )
                content_blocks.append(content_block)
            
            # Index the content blocks
            return self.index_book_content(book_id, content_blocks)
            
        except Exception as e:
            rag_logger.error(f"Error indexing text chunks for book {book_id}: {str(e)}")
            return False
    
    def _split_text_into_chunks(self, text: str, chunk_size: int) -> List[str]:
        """
        Split text into chunks of approximately chunk_size characters
        """
        chunks = []
        start = 0
        
        while start < len(text):
            end = start + chunk_size
            
            # Try to split at sentence boundary if possible
            if end < len(text):
                # Look for a sentence boundary near the end
                for i in range(end, min(end + 200, len(text))):
                    if text[i] in '.!?':
                        end = i + 1
                        break
            
            # If we couldn't find a good boundary, just split at chunk_size
            if start == end:  # This would happen if the chunk_size is smaller than a sentence
                end = start + chunk_size
            
            chunk = text[start:end].strip()
            if chunk:  # Only add non-empty chunks
                chunks.append(chunk)
            
            start = end
        
        return chunks
    
    def update_content_block(self, content_block: BookContent) -> bool:
        """
        Update a specific content block in the vector store
        """
        try:
            # Generate new embedding for the updated content
            embedding = embedding_service.generate_embedding(content_block.content)
            
            # Update the document in the vector store
            success = vector_store_service.add_document(
                doc_id=content_block.id,
                content=content_block.content,
                embedding=embedding,
                source_location=content_block.source_location,
                book_id=content_block.book_id
            )
            
            if success:
                rag_logger.info(f"Updated content block {content_block.id}")
            else:
                rag_logger.error(f"Failed to update content block {content_block.id}")
            
            return success
            
        except Exception as e:
            rag_logger.error(f"Error updating content block {content_block.id}: {str(e)}")
            return False
    
    def delete_content_block(self, content_id: str) -> bool:
        """
        Delete a specific content block from the vector store
        """
        try:
            success = vector_store_service.delete_document(content_id)
            
            if success:
                rag_logger.info(f"Deleted content block {content_id}")
            else:
                rag_logger.error(f"Failed to delete content block {content_id}")
            
            return success
            
        except Exception as e:
            rag_logger.error(f"Error deleting content block {content_id}: {str(e)}")
            return False


# Global instance
book_content_service = BookContentService()