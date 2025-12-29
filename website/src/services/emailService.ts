
import { API_CONFIG } from '../config/api';

// Function to send confirmation email after successful authentication
export const sendConfirmationEmail = async (email: string, name: string): Promise<void> => {
  try {
    const response = await fetch(`${API_CONFIG.BASE_URL}/v1/email/send-confirmation`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ 
        email, 
        name 
      }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Failed to send confirmation email');
    }

    console.log('Confirmation email sent successfully');
  } catch (error) {
    console.error('Error sending confirmation email:', error);
    // Don't throw error as this is not critical for the signup process
  }
};