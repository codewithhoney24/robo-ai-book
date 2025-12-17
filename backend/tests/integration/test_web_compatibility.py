import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch
from src.api.main import app
from src.models.generated_response import GeneratedResponse
from src.models.retrieved_context import RetrievedContext


client = TestClient(app)


def test_api_response_format_consistency():
    """
    Integration test to verify API response format consistency across different scenarios
    """
    with patch('src.api.chatbot_router.rag_service') as mock_rag_service:
        # Mock a consistent response
        mock_response = GeneratedResponse(
            id="response_1",
            content="Test response content",
            query_id="query_1",
            relevance_score=0.9,
            retrieved_contexts=[
                RetrievedContext(
                    id="context_1",
                    content="Test context",
                    relevance_score=0.85,
                    source_location="Chapter 1",
                    query_id="query_1"
                )
            ]
        )
        mock_rag_service.process_query_with_rag.return_value = mock_response

        # Test /chat endpoint
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "Test query?",
                "session_id": "test_session"
            },
            headers={"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"}
        )
        
        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert "relevance_score" in data
        assert "retrieved_contexts" in data
        
        # Test /chat/selected-text endpoint
        response2 = client.post(
            "/api/v1/chat/selected-text",
            json={
                "query": "Test query?",
                "selected_text": "Selected text",
                "session_id": "test_session"
            },
            headers={"User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X)"}
        )
        
        assert response2.status_code == 200
        data2 = response2.json()
        assert "response" in data2
        assert "relevance_score" in data2
        assert "retrieved_contexts" in data2


def test_cross_platform_headers_handling():
    """
    Integration test for handling different User-Agent headers
    """
    with patch('src.api.chatbot_router.rag_service') as mock_rag_service:
        mock_response = GeneratedResponse(
            id="response_1",
            content="Response for cross-platform test",
            query_id="query_1",
            relevance_score=0.85,
            retrieved_contexts=[]
        )
        mock_rag_service.process_query_with_rag.return_value = mock_response

        # Test with different User-Agent headers to ensure compatibility
        user_agents = [
            "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36",
            "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36",
            "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36",
            "Mozilla/5.0 (iPhone; CPU iPhone OS 14_6 like Mac OS X)",
            "Mozilla/5.0 (Android 11; Mobile; rv:90.0) Gecko/90.0"
        ]
        
        for ua in user_agents:
            response = client.post(
                "/api/v1/chat",
                json={
                    "query": "Platform compatibility test",
                    "session_id": f"session_{hash(ua) % 10000}"
                },
                headers={"User-Agent": ua}
            )
            
            # All requests should be successful regardless of User-Agent
            assert response.status_code == 200, f"Failed for User-Agent: {ua}"
            
            data = response.json()
            # Verify the response has the expected structure
            assert "response" in data
            assert "relevance_score" in data
            assert "retrieved_contexts" in data


def test_different_content_types():
    """
    Integration test to ensure API handles different content types properly
    """
    with patch('src.api.chatbot_router.rag_service') as mock_rag_service:
        mock_response = GeneratedResponse(
            id="response_1",
            content="Response for content-type test",
            query_id="query_1",
            relevance_score=0.8,
            retrieved_contexts=[]
        )
        mock_rag_service.process_query_with_rag.return_value = mock_response

        # Test with application/json content type (default)
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "Content type test",
                "session_id": "test_session_json"
            },
            headers={"Content-Type": "application/json"}
        )
        
        assert response.status_code == 200
        data = response.json()
        assert "response" in data