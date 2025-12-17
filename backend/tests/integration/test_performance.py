import pytest
import time
from fastapi.testclient import TestClient
from unittest.mock import patch
from src.api.main import app
from src.models.generated_response import GeneratedResponse
from src.models.retrieved_context import RetrievedContext


client = TestClient(app)


def test_response_time_under_threshold():
    """
    Performance test to ensure response time is under the required threshold
    """
    with patch('src.api.chatbot_router.rag_service') as mock_rag_service:
        # Create a mock response
        mock_response = GeneratedResponse(
            id="response_1",
            content="This is a test response for performance evaluation.",
            query_id="query_1",
            relevance_score=0.95,
            retrieved_contexts=[
                RetrievedContext(
                    id="context_1",
                    content="Sample context for performance testing",
                    relevance_score=0.85,
                    source_location="Chapter 1, Section 1",
                    query_id="query_1"
                )
            ]
        )
        mock_rag_service.process_query_with_rag.return_value = mock_response

        # Measure response time for /chat endpoint
        start_time = time.time()
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "What is the performance threshold?",
                "session_id": "perf_test_session"
            }
        )
        end_time = time.time()
        
        response_time = (end_time - start_time) * 1000  # Convert to milliseconds
        
        # Assert that response time is under 1000ms (1 second) - the performance goal
        assert response_time < 1000, f"Response time {response_time}ms exceeds 1000ms threshold"
        assert response.status_code == 200


def test_concurrent_request_handling():
    """
    Performance test to evaluate handling of concurrent requests
    """
    import threading
    import requests
    
    # This test simulates multiple concurrent requests to evaluate performance
    results = []
    
    def make_request(query_num):
        with patch('src.api.chatbot_router.rag_service') as mock_rag_service:
            mock_response = GeneratedResponse(
                id=f"response_{query_num}",
                content=f"Response for concurrent request {query_num}",
                query_id=f"query_{query_num}",
                relevance_score=0.85,
                retrieved_contexts=[]
            )
            mock_rag_service.process_query_with_rag.return_value = mock_response
            
            start_time = time.time()
            response = client.post(
                "/api/v1/chat",
                json={
                    "query": f"Concurrent request {query_num}",
                    "session_id": f"concurrent_session_{query_num}"
                }
            )
            end_time = time.time()
            
            results.append({
                'response_time': (end_time - start_time) * 1000,
                'status_code': response.status_code
            })
    
    # Create and start multiple threads to simulate concurrent requests
    threads = []
    num_requests = 5
    
    for i in range(num_requests):
        thread = threading.Thread(target=make_request, args=(i,))
        threads.append(thread)
        thread.start()
    
    # Wait for all threads to complete
    for thread in threads:
        thread.join()
    
    # Verify all requests were successful
    assert len(results) == num_requests
    for i, result in enumerate(results):
        assert result['status_code'] == 200, f"Request {i} failed with status {result['status_code']}"
        assert result['response_time'] < 2000, f"Request {i} took too long: {result['response_time']}ms"


def test_large_query_response_time():
    """
    Performance test with a larger query to evaluate how it affects response time
    """
    with patch('src.api.chatbot_router.rag_service') as mock_rag_service:
        # Create a longer query to test performance with more complex inputs
        long_query = "Explain in detail the complex concepts of Retrieval-Augmented Generation systems, " \
                     "including their architecture, implementation details, performance considerations, " \
                     "and practical applications in the context of modern AI systems, and how they " \
                     "compare to traditional language models in terms of accuracy, efficiency, and " \
                     "factual correctness? " * 5  # Repeat to make it longer
        
        mock_response = GeneratedResponse(
            id="response_long",
            content="Detailed response to the complex query about RAG systems.",
            query_id="long_query_1",
            relevance_score=0.92,
            retrieved_contexts=[
                RetrievedContext(
                    id="ctx_1",
                    content="RAG combines retrieval and generation for more accurate responses",
                    relevance_score=0.89,
                    source_location="Chapter 3, Section 2",
                    query_id="long_query_1"
                )
            ] * 3  # Multiple contexts
        )
        mock_rag_service.process_query_with_rag.return_value = mock_response

        start_time = time.time()
        response = client.post(
            "/api/v1/chat",
            json={
                "query": long_query,
                "session_id": "long_query_session"
            }
        )
        end_time = time.time()
        
        response_time = (end_time - start_time) * 1000
        
        # Even with a longer query, response should still be under 2 seconds
        assert response_time < 2000, f"Response time for long query {response_time}ms exceeds 2000ms threshold"
        assert response.status_code == 200


def test_selected_text_endpoint_performance():
    """
    Performance test specifically for the selected-text endpoint
    """
    with patch('src.api.chatbot_router.rag_service') as mock_rag_service:
        mock_response = GeneratedResponse(
            id="response_selected",
            content="Response based on selected text",
            query_id="selected_query_1",
            relevance_score=0.88,
            retrieved_contexts=[
                RetrievedContext(
                    id="ctx_selected",
                    content="Selected text context",
                    relevance_score=0.85,
                    source_location="Chapter 2, Page 45",
                    query_id="selected_query_1"
                )
            ]
        )
        mock_rag_service.process_query_with_rag.return_value = mock_response

        selected_text = "Retrieval-Augmented Generation (RAG) is a technique that combines information retrieval " \
                        "with text generation to provide more accurate and factual responses. It works by first " \
                        "searching a knowledge base for relevant information, then conditioning a language model " \
                        "on this information to generate a response."
        
        start_time = time.time()
        response = client.post(
            "/api/v1/chat/selected-text",
            json={
                "query": "Explain the selected concept in more depth",
                "selected_text": selected_text,
                "session_id": "selected_text_session"
            }
        )
        end_time = time.time()
        
        response_time = (end_time - start_time) * 1000
        
        # Response should be under 1500ms for selected-text endpoint
        assert response_time < 1500, f"Selected text response time {response_time}ms exceeds 1500ms threshold"
        assert response.status_code == 200