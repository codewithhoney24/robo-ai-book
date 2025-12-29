import smtplib
import logging
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from typing import Optional
from src.config.settings import settings


class EmailService:
    """
    A service class to handle email operations
    """
    
    @staticmethod
    def send_email(
        to_email: str,
        subject: str,
        body: str,
        from_email: Optional[str] = None,
        content_type: str = "plain"
    ) -> bool:
        """
        Send an email using SMTP
        
        Args:
            to_email: Recipient's email address
            subject: Email subject
            body: Email body content
            from_email: Sender's email address (uses default if not provided)
            content_type: Type of content (plain or html)
        
        Returns:
            bool: True if email was sent successfully, False otherwise
        """
        try:
            # Use default sender email if not provided
            sender_email = from_email or settings.sender_email or "noreply@yourdomain.com"
            
            # Create message
            msg = MIMEMultipart()
            msg['From'] = sender_email
            msg['To'] = to_email
            msg['Subject'] = subject

            # Add body to email
            msg.attach(MIMEText(body, content_type))

            # Connect to server and send email
            # In production, you should use a proper email service like SendGrid, AWS SES, etc.
            # For now, we'll log the email to simulate sending
            logging.info(f"Email sent to: {to_email}")
            logging.info(f"Subject: {subject}")
            logging.info(f"Body: {body}")

            # Uncomment the following code to actually send emails via SMTP:
            # server = smtplib.SMTP(settings.smtp_server, settings.smtp_port)
            # server.starttls()
            # server.login(settings.sender_email, settings.sender_password)
            # text = msg.as_string()
            # server.sendmail(settings.sender_email, to_email, text)
            # server.quit()

            return True
        except Exception as e:
            logging.error(f"Failed to send email to {to_email}: {str(e)}")
            return False

    @staticmethod
    def send_welcome_email(to_email: str, name: str) -> bool:
        """
        Send a welcome/confirmation email to a new user
        
        Args:
            to_email: Recipient's email address
            name: Recipient's name
        
        Returns:
            bool: True if email was sent successfully, False otherwise
        """
        subject = "Welcome! Your Account Has Been Created Successfully"
        body = f"""
Hi {name},

Your account has been successfully created on our platform.

Thank you for joining us! We're excited to have you as part of our community.

If you have any questions, feel free to reach out to our support team.

Best regards,
The Team
        """
        
        return EmailService.send_email(to_email, subject, body)

    @staticmethod
    def send_custom_email(to_email: str, name: str, subject: str, message: str) -> bool:
        """
        Send a custom email to a user
        
        Args:
            to_email: Recipient's email address
            name: Recipient's name
            subject: Email subject
            message: Custom message content
        
        Returns:
            bool: True if email was sent successfully, False otherwise
        """
        body = f"""
Hi {name},

{message}

Best regards,
The Team
        """
        
        return EmailService.send_email(to_email, subject, body)