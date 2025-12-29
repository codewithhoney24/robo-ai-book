from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from src.config.settings import settings
from src.services.email_service import EmailService
import logging

router = APIRouter(prefix="/email", tags=["email"])

# Pydantic models for email requests
class EmailRequest(BaseModel):
    email: str
    name: str

class EmailRequestWithSubject(BaseModel):
    email: str
    name: str
    subject: str
    message: str

def send_welcome_email_background(email: str, name: str):
    """
    Background task to send welcome email
    """
    EmailService.send_welcome_email(email, name)

def send_custom_email_background(email: str, name: str, subject: str, message: str):
    """
    Background task to send custom email
    """
    EmailService.send_custom_email(email, name, subject, message)

@router.post("/send-confirmation")
async def send_confirmation_email(request: EmailRequest, background_tasks: BackgroundTasks):
    """
    Send a confirmation email to the user after successful authentication
    """
    try:
        # Add email sending to background tasks
        background_tasks.add_task(send_welcome_email_background, request.email, request.name)
        return {"message": "Confirmation email sent successfully"}
    except Exception as e:
        logging.error(f"Failed to queue confirmation email: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to send confirmation email: {str(e)}")


@router.post("/send-custom")
async def send_custom_email(request: EmailRequestWithSubject, background_tasks: BackgroundTasks):
    """
    Send a custom email to the user
    """
    try:
        # Add email sending to background tasks
        background_tasks.add_task(send_custom_email_background, request.email, request.name, request.subject, request.message)
        return {"message": "Custom email sent successfully"}
    except Exception as e:
        logging.error(f"Failed to queue custom email: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to send custom email: {str(e)}")