import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatWidget.module.css'; // Assuming you'll create a CSS module for styling

interface ChatMessage {
  id: string;
  text: string;
  isUser: boolean;
  citations?: { chapter: string; section: string; file_path: string }[];
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom of chat messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(event.target.value);
  };

  const handleSendMessage = async (event: React.FormEvent) => {
    event.preventDefault();
    if (inputValue.trim() === '') return;

    const newUserMessage: ChatMessage = {
      id: Date.now().toString(),
      text: inputValue,
      isUser: true,
    };
    setMessages((prevMessages) => [...prevMessages, newUserMessage]);
    setInputValue('');
    setIsLoading(true);

    // --- Placeholder Backend Call ---
    // In a real implementation, this would call your FastAPI backend
    try {
      const response = await fetch('/api/v1/chat', { // Assuming backend is proxied or directly accessible
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: newUserMessage.text }),
      });

      const data = await response.json();

      const botResponse: ChatMessage = {
        id: Date.now().toString() + '-bot',
        text: data.response || 'Error: Could not get a response.',
        isUser: false,
        citations: data.citations,
      };
      setMessages((prevMessages) => [...prevMessages, botResponse]);
    } catch (error) {
      console.error('Error sending message to backend:', error);
      setMessages((prevMessages) => [
        ...prevMessages,
        {
          id: Date.now().toString() + '-error',
          text: 'Sorry, I am having trouble connecting to the server.',
          isUser: false,
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`${styles.chatWidget} ${isOpen ? styles.open : ''}`}>
      <button className={styles.toggleButton} onClick={toggleChat}>
        {isOpen ? 'Close Chat' : 'Chat with AI'}
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>AI Chatbot</h3>
            <button onClick={toggleChat}>X</button>
          </div>
          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <p className={styles.welcomeMessage}>Ask me anything about the textbook content!</p>
            )}
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.chatMessage} ${message.isUser ? styles.userMessage : styles.botMessage}`}
              >
                <p>{message.text}</p>
                {message.citations && message.citations.length > 0 && (
                  <div className={styles.citations}>
                    <strong>Citations:</strong>
                    <ul>
                      {message.citations.map((citation, index) => (
                        <li key={index}>
                          {citation.chapter}, {citation.section}
                          {citation.file_path && (
                            <a href={`/${citation.file_path.replace('humanoid-robotics-textbook/docs/', '')}`} target="_blank" rel="noopener noreferrer"> (link)</a>
                          )}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={styles.loadingMessage}>
                <div className={styles.spinner}></div>
                <p>Thinking...</p>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSendMessage} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Type your message..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
