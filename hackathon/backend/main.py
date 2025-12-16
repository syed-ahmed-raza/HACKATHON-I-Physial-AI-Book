import os
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware # ✅ IMPORT THIS
from pydantic import BaseModel
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from sentence_transformers import SentenceTransformer
import google.generativeai as genai

# Setup... (Same as before)
load_dotenv()
api_key = os.getenv("GEMINI_API_KEY")
if not api_key: exit()
genai.configure(api_key=api_key)

# Auto-Detect Model... (Same as before)
try:
    available_models = [m.name for m in genai.list_models() if 'generateContent' in m.supported_generation_methods]
    SELECTED_MODEL = available_models[0] if available_models else 'gemini-1.5-flash'
except:
    SELECTED_MODEL = 'gemini-1.5-flash'

qdrant_client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
COLLECTION_NAME = "physical_ai_book"
embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

app = FastAPI()

# ✅ ENABLE CORS (Ye React ko allow karega)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Development ke liye sab allow kar rahe hain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    query: str

@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    # ... (Baaki code bilkul same rahega jo pehle tha)
    # ...
    # ... Copy paste old logic here ...
    
    # Short version for context:
    try:
        vector = embedding_model.encode(request.query).tolist()
        search_result = qdrant_client.search(collection_name=COLLECTION_NAME, query_vector=vector, limit=3, with_payload=True)
        context = "\n".join([r.payload.get('page_content', '') for r in search_result])
        if not context.strip(): return {"response": "Textbook mein kuch nahi mila."}
        
        prompt = f"Context:\n{context}\n\nQuestion: {request.query}\nAnswer (Short):"
        model = genai.GenerativeModel(SELECTED_MODEL)
        response = model.generate_content(prompt)
        return {"response": response.text.strip()}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)