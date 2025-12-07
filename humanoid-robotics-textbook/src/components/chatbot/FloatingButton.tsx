import React from 'react';
import styles from './FloatingButton.module.css'; // Assuming a CSS module for styling

interface FloatingButtonProps {
  onClick: () => void;
  isOpen: boolean;
}

const FloatingButton: React.FC<FloatingButtonProps> = ({ onClick, isOpen }) => {
  return (
    <button
      className={`${styles.floatingButton} ${isOpen ? styles.hidden : ''}`}
      onClick={onClick}
      aria-label="Open Chatbot"
    >
      💬
    </button>
  );
};

export default FloatingButton;
