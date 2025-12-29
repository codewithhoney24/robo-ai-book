import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import patch
from src.api.main import app
from src.services.email_service import EmailService

client = TestClient(app)

def test_send_confirmation_email():
    """Test sending a confirmation email"""
    with patch.object(EmailService, 'send_welcome_email', return_value=True) as mock_send:
        response = client.post(
            "/api/v1/email/send-confirmation",
            json={
                "email": "test@example.com",
                "name": "Test User"
            }
        )
        
        assert response.status_code == 200
        assert response.json() == {"message": "Confirmation email sent successfully"}
        mock_send.assert_called_once_with("test@example.com", "Test User")


def test_send_custom_email():
    """Test sending a custom email"""
    with patch.object(EmailService, 'send_custom_email', return_value=True) as mock_send:
        response = client.post(
            "/api/v1/email/send-custom",
            json={
                "email": "test@example.com",
                "name": "Test User",
                "subject": "Test Subject",
                "message": "Test message content"
            }
        )
        
        assert response.status_code == 200
        assert response.json() == {"message": "Custom email sent successfully"}
        mock_send.assert_called_once_with("test@example.com", "Test User", "Test Subject", "Test message content")


def test_email_service_send_welcome():
    """Test the email service directly"""
    result = EmailService.send_welcome_email("test@example.com", "Test User")
    assert result is True


def test_email_service_send_custom():
    """Test the email service directly"""
    result = EmailService.send_custom_email(
        "test@example.com", 
        "Test User", 
        "Test Subject", 
        "Test message content"
    )
    assert result is True