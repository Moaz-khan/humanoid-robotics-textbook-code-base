# Quick Start Guide - Interactive Systems (Phase 2)

This guide provides instructions for setting up a local development environment for the Phase 2 interactive systems (RAG chatbot backend and frontend widget).

## Prerequisites

- **Python 3.10+**: With `pip` and `venv`.
- **Node.js**: Version 18.x or higher, with `npm` or `yarn`.
- **Docker**: For running local instances of Postgres and Qdrant (optional, but recommended).

## Backend (FastAPI) Setup

1.  **Navigate to the Backend Directory**:
    ```bash
    cd backend
    ```

2.  **Create and Activate a Virtual Environment**:
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```
    *(Note: `requirements.txt` will be created in a later task.)*

4.  **Set Up Environment Variables**:
    Create a `.env` file in the `backend/` directory with the following variables:
    ```
    OPENAI_API_KEY=...
    NEON_DATABASE_URL=...
    QDRANT_API_KEY=...
    QDRANT_HOST=...
    BETTER_AUTH_SECRET_KEY=...
    ```

5.  **Run the Backend Server**:
    ```bash
    uvicorn src.main:app --reload
    ```
    The backend API will be available at `http://localhost:8000`.

## Frontend (Docusaurus Widget) Setup

1.  **Navigate to the Docusaurus Project**:
    ```bash
    cd humanoid-robotics-textbook
    ```

2.  **Install Dependencies**:
    ```bash
    npm install
    ```

3.  **Start the Development Server**:
    ```bash
    npm run start
    ```
    The Docusaurus site, with the integrated chat widget, will be available at `http://localhost:3000`.