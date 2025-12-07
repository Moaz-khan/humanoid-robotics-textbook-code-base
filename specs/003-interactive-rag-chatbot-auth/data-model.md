# Data Model (Schema)

This document defines the database schema for the Neon Serverless Postgres database and the structure of the Qdrant vector collections.

## Neon DB Schema

### `users` table
- `user_id` (UUID, Primary Key)
- `email` (VARCHAR, Unique)
- `password_hash` (VARCHAR)
- `created_at` (TIMESTAMP)

### `user_background` table
- `background_id` (UUID, Primary Key)
- `user_id` (UUID, Foreign Key to `users.user_id`)
- `software_experience` (VARCHAR)
- `hardware_experience` (VARCHAR)
- `robotics_familiarity` (VARCHAR)
- `ai_ml_familiarity` (VARCHAR)

### `chat_history` table
- `message_id` (UUID, Primary Key)
- `session_id` (UUID, Foreign Key to `sessions.session_id`)
- `user_id` (UUID, Foreign Key to `users.user_id`, Nullable)
- `message_text` (TEXT)
- `response_text` (TEXT)
- `citations` (JSONB)
- `timestamp` (TIMESTAMP)

### `sessions` table
- `session_id` (UUID, Primary Key)
- `user_id` (UUID, Foreign Key to `users.user_id`, Nullable)
- `created_at` (TIMESTAMP)

## Qdrant Vector Collections

### `book_chunks` collection
- **Description**: Stores embeddings for the entire textbook content, chunked for efficient retrieval.
- **Fields**:
  - `vector`: The embedding vector.
  - `payload`:
    - `chapter`: String (e.g., "Module 1: ROS 2")
    - `section`: String (e.g., "Core Concepts of ROS 2")
    - `file_path`: String
    - `content`: String (The original text chunk)

### `selected_chunks` collection
- **Description**: A temporary, per-query collection used for the "Answer from selected text only" mode.
- **Fields**:
  - `vector`: The embedding vector.
  - `payload`:
    - `content`: String (The user-highlighted text)
- **Note**: This collection will be created on-the-fly for each selection-based query and can be deleted after the query is complete to conserve resources.
