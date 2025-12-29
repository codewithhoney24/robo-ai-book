from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import BaseModel
from typing import Optional
from src.database.connection import get_db_session
from src.models.user import User
from sqlalchemy.ext.asyncio import AsyncSession
import os
from openai import OpenAI
import logging

router = APIRouter(prefix="/personalization", tags=["personalization"])

logger = logging.getLogger(__name__)

class PersonalizationRequest(BaseModel):
    userId: str
    content: str
    userBackground: dict
    chapterTitle: Optional[str] = None

class PersonalizationResponse(BaseModel):
    personalizedContent: str

@router.post("/personalize", response_model=PersonalizationResponse)
async def personalize_content(
    request: PersonalizationRequest,
    db: AsyncSession = Depends(get_db_session)
):
    try:
        # Get user from database to verify they exist
        from sqlalchemy import select
        result = await db.execute(select(User).filter(User.id == request.userId))
        user = result.scalar_one_or_none()
        if not user:
            raise HTTPException(status_code=404, detail="User not found")

        # Get OpenAI API key from environment
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise HTTPException(status_code=500, detail="OpenAI API key not configured")

        # Create OpenAI client
        client = OpenAI(api_key=openai_api_key)

        # Create a prompt for the LLM based on user background
        prompt = f"""
        Personalize the following content based on the user's background:

        User's Software Background: {request.userBackground.get('softwareBackground', 'Not specified')}
        User's Hardware Background: {request.userBackground.get('hardwareBackground', 'Not specified')}

        Chapter Title: {request.chapterTitle or 'Not specified'}

        Original Content:
        {request.content}

        Please adapt the content to match the user's experience level and hardware availability.
        For example:
        - If the user has a beginner software background, explain concepts more simply and provide more foundational context
        - If the user has an intermediate software background, provide balanced explanations with some technical details
        - If the user has an advanced software background, include more technical details and advanced concepts
        - If the user has a python_intermediate background, focus on Python-based implementations
        - If the user has a ros2_developer background, include more advanced ROS 2 concepts
        - If the user has an ai_robotics_expert background, include research-level implementations
        - If the user has limited hardware (e.g., no_gpu), suggest cloud alternatives or warn about resource requirements
        - If the user has RTX laptop/workstation, include content about leveraging GPU capabilities
        - If the user has Jetson kit, focus on edge computing and embedded systems
        - If the user has cloud resources, include content about cloud robotics

        Return only the personalized content, maintaining the original structure and format as much as possible.
        """

        # Call OpenAI API to personalize content
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",  # You can change this to gpt-4 if preferred
            messages=[
                {"role": "system", "content": "You are an expert in robotics education and content personalization. Your task is to adapt educational content based on the user's background and experience level."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=2000,  # Adjust as needed
            temperature=0.7
        )

        personalized_content = response.choices[0].message.content.strip()

        return PersonalizationResponse(personalizedContent=personalized_content)

    except Exception as e:
        logger.error(f"Error in personalization endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error personalizing content: {str(e)}")