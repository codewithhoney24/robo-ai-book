from fastapi import APIRouter, HTTPException
from typing import List
from ..models.book_content import BookContent
from ..services.book_content_service import book_content_service
from ..config.logging_config import api_logger


router = APIRouter()


@router.post("/book-content/index")
async def index_book_content_endpoint(book_data: dict):
    """
    Endpoint to index book content for RAG retrieval
    """
    try:
        # Validate required fields
        book_id = book_data.get("book_id")
        if not book_id:
            raise HTTPException(
                status_code=422,
                detail={
                    "error": "book_id is required",
                    "required_fields": ["book_id", "content_blocks"],
                    "example_payload": {
                        "book_id": "unique-book-identifier",
                        "content_blocks": [
                            {
                                "id": "block-1",
                                "content": "The actual text content to be indexed",
                                "source_location": "chapter-1-page-1",
                                "book_id": "unique-book-identifier"
                            }
                        ]
                    }
                }
            )

        content_blocks_data = book_data.get("content_blocks")
        if content_blocks_data is None:
            raise HTTPException(
                status_code=422,
                detail={
                    "error": "content_blocks is required",
                    "required_fields": ["book_id", "content_blocks"],
                    "example_payload": {
                        "book_id": "unique-book-identifier",
                        "content_blocks": [
                            {
                                "id": "block-1",
                                "content": "The actual text content to be indexed",
                                "source_location": "chapter-1-page-1",
                                "book_id": "unique-book-identifier"
                            }
                        ]
                    }
                }
            )

        if not isinstance(content_blocks_data, list) or len(content_blocks_data) == 0:
            raise HTTPException(
                status_code=422,
                detail={
                    "error": "content_blocks must be a non-empty list",
                    "required_fields": ["book_id", "content_blocks"],
                    "example_payload": {
                        "book_id": "unique-book-identifier",
                        "content_blocks": [
                            {
                                "id": "block-1",
                                "content": "The actual text content to be indexed",
                                "source_location": "chapter-1-page-1",
                                "book_id": "unique-book-identifier"
                            }
                        ]
                    }
                }
            )

        # Create BookContent objects from the data
        content_blocks = []
        for i, block_data in enumerate(content_blocks_data):
            if not isinstance(block_data, dict):
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": f"content_blocks[{i}] must be an object",
                        "required_fields": ["id", "content", "source_location", "book_id"],
                        "example_payload": {
                            "book_id": "unique-book-identifier",
                            "content_blocks": [
                                {
                                    "id": "block-1",
                                    "content": "The actual text content to be indexed",
                                    "source_location": "chapter-1-page-1",
                                    "book_id": "unique-book-identifier"
                                }
                            ]
                        }
                    }
                )

            try:
                content_block = BookContent(**block_data)
                content_blocks.append(content_block)
            except Exception as validation_error:
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": f"Invalid content block at index {i}: {str(validation_error)}",
                        "required_fields": ["id", "content", "source_location", "book_id"],
                        "example_payload": {
                            "book_id": "unique-book-identifier",
                            "content_blocks": [
                                {
                                    "id": "block-1",
                                    "content": "The actual text content to be indexed",
                                    "source_location": "chapter-1-page-1",
                                    "book_id": "unique-book-identifier"
                                }
                            ]
                        }
                    }
                )

        # Index the content
        success = book_content_service.index_book_content(book_id, content_blocks)

        if success:
            api_logger.info(f"Successfully indexed book content for book {book_id}")
            return {"message": f"Successfully indexed {len(content_blocks)} content blocks for book {book_id}", "success": True}
        else:
            api_logger.error(f"Failed to index book content for book {book_id}")
            raise HTTPException(status_code=500, detail="Failed to index book content")

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise

    except Exception as e:
        api_logger.error(f"Unexpected error in index book content endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="An unexpected error occurred while indexing book content")


@router.post("/book-content/index-text")
async def index_text_chunks_endpoint(book_data: dict):
    """
    Endpoint to split book text into chunks and index them for RAG retrieval
    """
    try:
        book_id = book_data.get("book_id")
        full_text = book_data.get("full_text")

        if not book_id:
            raise HTTPException(status_code=422, detail="book_id is required")

        if not full_text:
            # Check if selected_text is provided as a fallback
            selected_text = book_data.get("selected_text", "").strip()
            if selected_text:
                full_text = selected_text
            else:
                raise HTTPException(status_code=422, detail="full_text is required")

        # Get chunk size from request or use default
        chunk_size = book_data.get("chunk_size", 1000)

        # Index the text chunks
        success = book_content_service.index_text_chunks(book_id, full_text, chunk_size)

        if success:
            api_logger.info(f"Successfully indexed text chunks for book {book_id}")
            return {"message": f"Successfully indexed text for book {book_id}", "success": True}
        else:
            api_logger.error(f"Failed to index text chunks for book {book_id}")
            raise HTTPException(status_code=500, detail="Failed to index text chunks")

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise

    except Exception as e:
        api_logger.error(f"Unexpected error in index text chunks endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="An unexpected error occurred while indexing text chunks")