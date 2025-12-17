import os
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from pathlib import Path
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import google.generativeai as genai

# --- 🛠️ 1. SETUP ENVIRONMENT (FIXED PATH) ---
# Ye logic local terminal aur Render dono ke liye best hai
env_path = Path(__file__).resolve().parent / ".env"
load_dotenv(dotenv_path=env_path)

# Key load karte waqt prioritize environment variables
api_key = os.environ.get("GEMINI_API_KEY") or os.getenv("GEMINI_API_KEY")

if not api_key:
    print("❌ Error: GEMINI_API_KEY not found! Checking hardcoded fallback...")
    # Agar bilkul na chale toh niche wali line temporarily use karein:
    # api_key = "AIzaSyDFAodhJg2eItCWvSCvJ1Jhz6pK-UNSHe4"

if api_key:
    print(f"✅ GEMINI_API_KEY Found: {api_key[:5]}...")
    genai.configure(api_key=api_key)
else:
    print("🛑 Critical Error: No API Key available. Exiting.")
    exit()

# --- 🧠 SMART MODEL SELECTOR ---
def get_working_model():
    candidates = [
        "gemini-1.5-flash",
        "gemini-1.5-flash-001",
        "gemini-1.5-pro",
        "gemini-pro"
    ]
    
    print("🔎 Testing available models...")
    for model_name in candidates:
        try:
            model = genai.GenerativeModel(model_name)
            model.generate_content("Hi", generation_config={"max_output_tokens": 5})
            print(f" ✅ {model_name} Works!")
            return model_name
        except Exception:
            print(f" ❌ {model_name} Failed")
    
    # Fallback to list
    try:
        for m in genai.list_models():
            if 'generateContent' in m.supported_generation_methods:
                return m.name
    except:
        pass
    
    return "gemini-1.5-flash" # Absolute last resort

# Best Model select karlo
SELECTED_MODEL = "gemini-1.5-flash"
try:
    SELECTED_MODEL = get_working_model()
except Exception as e:
    print(f"⚠️ Model test failed: {e}")

print(f"🚀 Final Model Choice: {SELECTED_MODEL}")

# --- 2. DATABASE SETUP ---
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
COLLECTION_NAME = "physical_ai_book"

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    query: str

@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    try:
        # A. Embed Query
        emb_result = genai.embed_content(
            model="models/text-embedding-004",
            content=request.query,
            task_type="retrieval_query"
        )
        query_vector = emb_result['embedding']

        # B. Search Qdrant
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=3,
            with_payload=True
        )

        # C. Context
        context = "\n".join([r.payload.get('page_content', '') for r in search_result])
        if not context.strip():
            context = "Information not found in specific context."

        # D. Generate Answer
        prompt = f"Context from book:\n{context}\n\nUser Question: {request.query}\nAnswer (Keep it short and helpful):"
        model = genai.GenerativeModel(SELECTED_MODEL)
        response = model.generate_content(prompt)
        
        return {"response": response.text.strip()}

    except Exception as e:
        print(f"❌ API Error: {e}")
        return {"response": "Sorry, I encountered an error. Please try again."}

if __name__ == "__main__":
    # Render automatically uses the PORT env var
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)