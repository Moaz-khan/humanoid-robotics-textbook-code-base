# Quick Start Guide - Physical AI & Humanoid Robotics Textbook

This guide provides instructions for setting up a local development environment to build and preview the "Physical AI & Humanoid Robotics" textbook website.

## Prerequisites

- **Node.js**: Version 18.x or higher.
- **npm** or **yarn**: A modern version of the package manager.
- **Git**: For cloning the repository.

## Local Development Setup

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/your-github-username/physical-ai-humanoid-robotics.git
    cd physical-ai-humanoid-robotics
    ```

2.  **Navigate to the Docusaurus Project**:
    The Docusaurus project is located in the `humanoid-robotics-textbook/` directory.
    ```bash
    cd humanoid-robotics-textbook
    ```

3.  **Install Dependencies**:
    Using npm:
    ```bash
    npm install
    ```
    Or using yarn:
    ```bash
    yarn install
    ```

4.  **Start the Development Server**:
    This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.
    ```bash
    npm run start
    ```
    Or using yarn:
    ```bash
    yarn start
    ```
    By default, the site will be available at `http://localhost:3000`.

## Building the Static Site

To generate a production-ready static build of the website:

1.  **Run the build command**:
    ```bash
    npm run build
    ```
    Or using yarn:
    ```bash
    yarn build
    ```

2.  **Locate the build output**:
    The static files will be generated in the `build/` directory. These are the files that will be deployed to GitHub Pages.

## Serving the Static Site Locally

After building the site, you can test the production build locally before deploying.

```bash
npm run serve
```
Or using yarn:
```bash
yarn serve
```
This will serve the content of the `build/` directory at `http://localhost:3000`.