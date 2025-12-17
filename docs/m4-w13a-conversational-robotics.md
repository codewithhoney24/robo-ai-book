# Module 4: Conversational Robotics (Wk 13 Part A)

## 1. Integrating GPT models for conversational AI in robots.

Integrating large language models (LLMs) like **GPT** into robotics enables more **natural and intelligent human-robot interaction**. This involves sending user queries to the LLM and then interpreting its response to generate robot actions.

### Example Code Snippet: Basic GPT Integration

```python
import openai

def get_gpt_response(prompt):
    response = openai.Completion.create(
        engine="text-davinci-003", # Or a more recent GPT model
        prompt=prompt,
        max_tokens=150
    )
    return response.choices[0].text.strip()

def robot_action_from_gpt(user_command):
    # This is a simplified example. Real-world systems would have more robust parsing.
    gpt_prompt = f"Given the command '{user_command}', what action should a robot take? Respond concisely with a verb and object, e.g., 'move forward' or 'pick up ball'."
    action_text = get_gpt_response(gpt_prompt)
    print(f"Robot interprets: {action_text}")
    # Further logic to map action_text to actual robot movements/functions
    if "move forward" in action_text.lower():
        print("Executing: Move robot forward")
    elif "pick up" in action_text.lower():
        print("Executing: Pick up object")
    # ... more action mappings

# Example usage:
# robot_action_from_gpt("Can you go to the kitchen?")