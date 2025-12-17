import sys
import os
sys.path.insert(0, os.getcwd())

from src.services.book_content_service import book_content_service
import json

# Load sample book content as it would be sent to the endpoint
with open('sample_book_content.json', 'r') as f:
    sample_content = json.load(f)

# Prepare content blocks in the same way as the endpoint
from src.models.book_content import BookContent
content_blocks = []
for i, block_data in enumerate(sample_content):
    try:
        content_block = BookContent(**block_data)
        content_blocks.append(content_block)
    except Exception as validation_error:
        print(f"Validation error for content block at index {i}: {str(validation_error)}")
        break

print(f"Successfully created {len(content_blocks)} BookContent objects")

# Now try to index the content
book_id = "ai_robotics_book"

try:
    success = book_content_service.index_book_content(book_id, content_blocks)
    
    if success:
        print("Successfully indexed book content!")
    else:
        print("Failed to index book content")
except Exception as e:
    print(f"Exception during indexing: {e}")
    import traceback
    traceback.print_exc()