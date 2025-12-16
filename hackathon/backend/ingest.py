import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer # ✅ Local Model

# 1. Setup Environment
load_dotenv()

# ✅ Local Model Load karo (Internet API ki zaroorat nahi)
print("📥 Loading Local AI Model (all-MiniLM-L6-v2)...")
model = SentenceTransformer('all-MiniLM-L6-v2') 

# 2. Connect to Qdrant
qdrant_url = os.getenv("QDRANT_URL")
qdrant_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url or not qdrant_key:
    print("❌ Error: Qdrant keys missing in .env")
    exit()

qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_key)
COLLECTION_NAME = "physical_ai_book"

# 3. Create Collection (Size 384 for Local Model)
try:
    # Purani collection delete karni padegi kyunki size change ho raha hai
    qdrant.delete_collection(COLLECTION_NAME)
    print(f"🗑️ Old collection deleted (Size mismatch fix).")
except:
    pass

print(f"⚙️ Creating new collection '{COLLECTION_NAME}'...")
qdrant.create_collection(
    collection_name=COLLECTION_NAME,
    vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
)

# 4. Chunking Function
def chunk_text(text, chunk_size=1000):
    chunks = []
    current_chunk = ""
    paragraphs = text.split('\n\n')
    for para in paragraphs:
        if len(current_chunk) + len(para) < chunk_size:
            current_chunk += para + "\n\n"
        else:
            if current_chunk: chunks.append(current_chunk.strip())
            current_chunk = para + "\n\n"
    if current_chunk: chunks.append(current_chunk.strip())
    return chunks

# 5. Process & Upload
print("🔍 Scanning Docs (Super Fast Local Mode)...")
docs_path = "../frontend/docs"
points = []
point_id = 1

for root, dirs, files in os.walk(docs_path):
    for file in files:
        if file.endswith(".md"):
            file_path = os.path.join(root, file)
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
            
            text_chunks = chunk_text(content)
            print(f"   📄 Processing: {file} ({len(text_chunks)} chunks)")

            for chunk in text_chunks:
                if not chunk.strip(): continue
                
                try:
                    # ✅ Local CPU se Embedding Banao (Super Fast)
                    embedding = model.encode(chunk).tolist()
                    
                    points.append(models.PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={"page_content": chunk, "source": file}
                    ))
                    point_id += 1
                except Exception as e:
                    print(f"⚠️ Error: {e}")

# 6. Upload Final Batch
if points:
    print(f"🚀 Uploading {len(points)} vectors to Qdrant...")
    try:
        qdrant.upsert(collection_name=COLLECTION_NAME, points=points)
        print("✅ Ingestion Complete! Chatbot brain is ready.")
    except Exception as e:
        print(f"❌ Upload Failed: {e}")
else:
    print("❌ No chunks processed.")