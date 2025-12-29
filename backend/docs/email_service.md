# Email Service Documentation

## Overview
The email service provides functionality for sending various types of emails within the application. It includes a service layer and API endpoints to handle email operations.

## Components

### 1. Email Service (`src/services/email_service.py`)

The `EmailService` class provides the core functionality for sending emails:

- `send_email()`: Generic method to send emails with customizable content
- `send_welcome_email()`: Sends a welcome/confirmation email to new users
- `send_custom_email()`: Sends a custom email with user-defined subject and message

### 2. Email API (`src/api/v1/email.py`)

The API provides endpoints for sending emails:

- `POST /api/v1/email/send-confirmation`: Sends a welcome email to a new user
- `POST /api/v1/email/send-custom`: Sends a custom email with specified subject and content

## Configuration

Email settings are configured in `src/config/settings.py`:

- `smtp_server`: SMTP server address (default: "smtp.gmail.com")
- `smtp_port`: SMTP server port (default: 587)
- `sender_email`: Email address for sending emails
- `sender_password`: Password for the sender email account

## Usage

### Sending a Confirmation Email

```python
# Example request to /api/v1/email/send-confirmation
{
  "email": "user@example.com",
  "name": "John Doe"
}
```

### Sending a Custom Email

```python
# Example request to /api/v1/email/send-custom
{
  "email": "user@example.com",
  "name": "John Doe",
  "subject": "Custom Subject",
  "message": "Custom message content"
}
```

## Implementation Details

- Emails are sent using background tasks to avoid blocking the main thread
- The service currently logs emails instead of sending them to avoid requiring SMTP credentials during development
- To enable actual email sending, uncomment the SMTP code in the email service
- The service supports both plain text and HTML email content

## Security Considerations

- Store SMTP credentials securely in environment variables
- Use app-specific passwords for email providers when possible
- Implement rate limiting to prevent email spam
- Validate email addresses before sending

## Testing

Run the tests to verify email functionality:

```bash
# Test the email service directly
python -m pytest tests/test_email_service.py

# Test the email API endpoints
python -m pytest tests/test_email_api.py
```