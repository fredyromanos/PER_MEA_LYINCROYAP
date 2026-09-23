import uvicorn
import os

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from config import SERVER_PORT
from routes.messages import router as messages_router

app = FastAPI()

# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],
#     allow_methods=["*"],
#     allow_headers=["*"],
# )


app.include_router(messages_router, prefix="/api")

@app.exception_handler(Exception)
async def exception_handler(request: Request, exc: Exception):
    return JSONResponse(status_code=500, content={
        "detail": str(exc)
    })


@app.get("/")
async def read_root():
    return RedirectResponse(url="/index.html")

static_folder = os.path.join(os.path.dirname(__file__), "static")
app.mount("/", StaticFiles(directory=static_folder, html=True), name="static")

if __name__ == "__main__":
    # Launch server
    uvicorn.run("webserver:app", host="0.0.0.0", port=SERVER_PORT, reload=True)
