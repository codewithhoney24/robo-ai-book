import pytest
from src.services.email_service import EmailService


def test_send_welcome_email():
    """Test sending a welcome email"""
    result = EmailService.send_welcome_email("test@example.com", "Test User")
    assert result is True


def test_send_custom_email():
    """Test sending a custom email"""
    result = EmailService.send_custom_email(
        "test@example.com",
        "Test User",
        "Test Subject",
        "Test message content"
    )
    assert result is True


def test_send_email_with_html():
    """Test sending an HTML email"""
    html_content = "<h1>Hello</h1><p>This is an HTML email</p>"
    result = EmailService.send_email(
        "test@example.com",
        "HTML Test Email",
        html_content,
        content_type="html"
    )
    assert result is True


def test_send_email_with_invalid_email():
    """Test sending an email with invalid email format"""
    # This should still return True since we're just logging in our implementation
    result = EmailService.send_email(
        "invalid-email",
        "Test Subject",
        "Test message"
    )
    # In our current implementation, it will still return True because we're just logging
    assert result is True