# import os
# import uvicorn
# from fastapi import FastAPI, HTTPException
# from fastapi.middleware.cors import CORSMiddleware # ✅ IMPORT THIS
# from pydantic import BaseModel
# from dotenv import load_dotenv
# from qdrant_client import QdrantClient
# from sentence_transformers import SentenceTransformer
# import google.generativeai as genai

# # Setup... (Same as before)
# load_dotenv()
# api_key = os.getenv("GEMINI_API_KEY")
# if not api_key: exit()
# genai.configure(api_key=api_key)

# # Auto-Detect Model... (Same as before)
# try:
#     available_models = [m.name for m in genai.list_models() if 'generateContent' in m.supported_generation_methods]
#     SELECTED_MODEL = available_models[0] if available_models else 'gemini-1.5-flash'
# except:
#     SELECTED_MODEL = 'gemini-1.5-flash'

# qdrant_client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
# COLLECTION_NAME = "physical_ai_book"
# embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

# app = FastAPI()

# # ✅ ENABLE CORS (Ye React ko allow karega)
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],  # Development ke liye sab allow kar rahe hain
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

# class ChatRequest(BaseModel):
#     query: str

# @app.post("/chat")
# async def chat_endpoint(request: ChatRequest):
#     # ... (Baaki code bilkul same rahega jo pehle tha)
#     # ...
#     # ... Copy paste old logic here ...
    
#     # Short version for context:
#     try:
#         vector = embedding_model.encode(request.query).tolist()
#         search_result = qdrant_client.search(collection_name=COLLECTION_NAME, query_vector=vector, limit=3, with_payload=True)
#         context = "\n".join([r.payload.get('page_content', '') for r in search_result])
#         if not context.strip(): return {"response": "Textbook mein kuch nahi mila."}
        
#         prompt = f"Context:\n{context}\n\nQuestion: {request.query}\nAnswer (Short):"
#         model = genai.GenerativeModel(SELECTED_MODEL)
#         response = model.generate_content(prompt)
#         return {"response": response.text.strip()}
#     except Exception as e:
#         raise HTTPException(status_code=500, detail=str(e))

# if __name__ == "__main__":
#     uvicorn.run(app, host="0.0.0.0", port=8000)

import os
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import google.generativeai as genai

# 1. Setup Environment
load_dotenv()
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    print("❌ Error: GEMINI_API_KEY missing.")
    exit()

genai.configure(api_key=api_key)

# --- 🧠 SMART MODEL SELECTOR ---
def get_working_model():
    # In models ko bari bari test karega
    candidates = [
        "gemini-1.5-flash",
        "gemini-1.5-flash-001",
        "gemini-1.5-pro",
        "gemini-pro",
        "gemini-1.0-pro"
    ]
    
    print("🔎 Testing available models...")
    for model_name in candidates:
        try:
            print(f"   Testing: {model_name}...", end=" ")
            model = genai.GenerativeModel(model_name)
            # Chota sa test
            model.generate_content("Hello")
            print("✅ Works!")
            return model_name
        except Exception as e:
            print("❌ Failed")
    
    print("⚠️ Koi bhi specific model nahi chala. Auto-list se try karte hain...")
    # Aakhri koshish: List se uthao
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            return m.name
            
    raise Exception("❌ No working Gemini model found. API Key check karein.")

# Best Model select karlo
SELECTED_MODEL = get_working_model()
print(f"🚀 Using Model: {SELECTED_MODEL}")
# ----------------------------------

# 2. Database Setup
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
        # A. Embed Query (Google API)
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
            # Fallback agar book mein na mile to general knowledge se answer de
            context = "Information not found in specific context."

        # D. Generate Answer
        prompt = f"Context from book:\n{context}\n\nUser Question: {request.query}\nAnswer (Keep it short and helpful):"
        
        model = genai.GenerativeModel(SELECTED_MODEL)
        response = model.generate_content(prompt)
        
        return {"response": response.text.strip()}

    except Exception as e:
        print(f"❌ Error: {e}")
        return {"response": "Sorry, I encountered an error. Please try again."}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)