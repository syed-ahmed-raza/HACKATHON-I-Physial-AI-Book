import requests
import json

# URL of your local server
url = "http://127.0.0.1:8000/chat"

# Question to ask the bot
payload = {
    "query": "What is ROS?"
}

try:
    print(f"🤖 Asking: {payload['query']}...")
    
    # Send request to server
    response = requests.post(url, json=payload)
    
    if response.status_code == 200:
        data = response.json()
        print("\n✅ AI Response:")
        print("------------------------------------------------")
        print(data.get("response", "No response field found."))
        print("------------------------------------------------")
    else:
        print(f"❌ Error: {response.status_code}")
        print(response.text)

except Exception as e:
    print(f"❌ Connection Failed: {e}")
    print("Make sure 'python main.py' is running in another terminal!")