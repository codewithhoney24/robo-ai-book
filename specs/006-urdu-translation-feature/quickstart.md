# Quickstart Guide: Urdu Translation Feature

## Prerequisites

- Node.js (for Docusaurus frontend)
- Python 3.11 (for FastAPI backend)
- OpenAI API key configured in environment variables
- Docusaurus project already set up

## 1. Backend Setup (FastAPI)

### Create the Translation Endpoint

Add the following to your FastAPI application (typically in `server.py` or appropriate route file):

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import openai
import os
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()

# Translation request/response models
class TranslationRequest(BaseModel):
    text: str

class TranslationResponse(BaseModel):
    translated_text: str

@app.post("/api/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    """
    Translate English text to Urdu while preserving technical terms in English.
    """
    if not request.text or len(request.text.strip()) == 0:
        raise HTTPException(status_code=400, detail="Text field is required and cannot be empty")
    
    # Call OpenAI API with specific system prompt
    try:
        # Load the OpenAI API key from environment variables
        openai.api_key = os.getenv("OPENAI_API_KEY")
        
        system_prompt = "You are a technical translator. Translate the text to Urdu. Keep technical terms (ROS 2, Gazebo, Isaac Sim, Python, Nodes, Topics) in English. Output only the translated text."
        
        response = openai.ChatCompletion.create(
            model="gpt-4o-mini",  # or another available model
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.text}
            ],
            max_tokens=2000,  # Adjust based on expected output length
            temperature=0.3
        )
        
        translated_text = response.choices[0].message.content.strip()
        logger.info(f"Successfully translated text of length {len(request.text)}")
        
        return TranslationResponse(translated_text=translated_text)
    
    except Exception as e:
        logger.error(f"Translation failed: {str(e)}")
        raise HTTPException(status_code=500, detail="Translation service failed")
```

### Environment Configuration

Ensure your `.env` file has the OpenAI API key:

```
OPENAI_API_KEY=your_openai_api_key_here
```

## 2. Frontend Setup (Docusaurus)

### Swizzle the DocItem Component

First, swizzle the DocItem component to wrap it:

```bash
npm run swizzle @docusaurus/theme-classic DocItem -- --wrap
```

This will create a `src/theme/DocItem/index.js` file in your Docusaurus project.

### Modify the Swizzled Component

Update the swizzled component (`src/theme/DocItem/index.js`) to add the translation functionality:

```javascript
import React, { useState } from 'react';
import OriginalDocItem from '@theme-original/DocItem';
import { translate } from '@docusaurus/core/lib/client/exports/Translate';

// Helper function to extract text from React children
function extractTextFromChildren(children) {
  let text = '';
  
  React.Children.forEach(children, (child) => {
    if (typeof child === 'string' || typeof child === 'number') {
      text += child + ' ';
    } else if (React.isValidElement(child)) {
      if (child.props.children) {
        text += extractTextFromChildren(child.props.children) + ' ';
      }
    }
  });
  
  return text.trim();
}

export default function DocItem(props) {
  const [urduText, setUrduText] = useState(null);
  const [isTranslating, setIsTranslating] = useState(false);

  const handleTranslate = async () => {
    setIsTranslating(true);
    try {
      // Extract content text from the original component
      const textContent = extractTextFromChildren(props.content);
      
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ text: textContent }),
      });

      if (!response.ok) {
        throw new Error('Translation request failed');
      }

      const data = await response.json();
      setUrduText(data.translated_text);
    } catch (error) {
      console.error('Translation error:', error);
      // Handle error appropriately
    } finally {
      setIsTranslating(false);
    }
  };

  const handleReset = () => {
    setUrduText(null);
  };

  // If we have Urdu text, show it with RTL styling
  if (urduText) {
    return (
      <div>
        <div className="urdu-translation-controls">
          <button onClick={handleReset} className="button button--secondary button--sm">
            Show Original
          </button>
        </div>
        <div dir="rtl" style={{ textAlign: 'right' }}>
          {urduText}
        </div>
      </div>
    );
  }

  return (
    <>
      <div className="translation-controls">
        <button 
          onClick={handleTranslate} 
          disabled={isTranslating}
          className="button button--primary button--sm"
        >
          {isTranslating ? 'Translating...' : 'Translate to Urdu'}
        </button>
      </div>
      <OriginalDocItem {...props} />
    </>
  );
}
```

### Add RTL Styling

Add the following to your Docusaurus CSS file (e.g., `src/css/custom.css`) to improve the RTL display:

```css
/* Right-to-left text styling for Urdu content */
[dir="rtl"] {
  text-align: right;
  direction: rtl;
}

/* Special styling for mixed English/Urdu content */
.urdu-content {
  direction: rtl;
  text-align: right;
}

/* Styling for translation controls */
.translation-controls {
  margin-bottom: 1rem;
  text-align: right;
}

.urdu-translation-controls {
  margin-bottom: 1rem;
  text-align: left;
}
```

## 3. Run the Application

### Backend
```bash
cd backend
pip install -r requirements.txt
python server.py
```

### Frontend
```bash
cd website  # or wherever your Docusaurus project is
npm install
npm run start
```

## 4. Testing

### Backend API Test
```bash
curl -X POST http://localhost:8000/api/translate \
  -H "Content-Type: application/json" \
  -d '{"text": "This is a test of the translation API. ROS 2 and Gazebo are important tools."}'
```

### Frontend Test
1. Navigate to any documentation page
2. Click the "Translate to Urdu" button
3. Verify that:
   - A "Translating..." state appears
   - The content is replaced with Urdu translation
   - Technical terms remain in English
   - Text is displayed with RTL alignment
4. Click "Show Original" to return to English content