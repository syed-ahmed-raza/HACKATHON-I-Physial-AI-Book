import React, { useState } from 'react';

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setMessages([{ sender: 'ai', text: 'Hello! How can I help you with the Physical AI textbook today?' }]);
    }
  };

  const handleSend = async () => {
    if (!input.trim()) return;

    const userMessage = { sender: 'user', text: input };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch('http://127.0.0.1:8000/chat', {
        method: 'POST',
        body: JSON.stringify({ query: input }),
        headers: { 'Content-Type': 'application/json' },
      });

      if (!response.ok) {
        throw new Error('Server not reachable');
      }

      const data = await response.json();
      const aiMessage = { sender: 'ai', text: data.response };
      setMessages((prev) => [...prev, aiMessage]);
    } catch (error) {
      const errorMessage = { sender: 'ai', text: 'Error: Could not connect to the server.' };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const styles = {
    chatButton: {
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      width: '60px',
      height: '60px',
      borderRadius: '50%',
      backgroundColor: '#007bff',
      color: 'white',
      display: 'flex',
      justifyContent: 'center',
      alignItems: 'center',
      cursor: 'pointer',
      fontSize: '24px',
      zIndex: 1000,
    },
    chatWindow: {
      position: 'fixed',
      bottom: '90px',
      right: '20px',
      width: '300px',
      height: '400px',
      backgroundColor: 'white',
      borderRadius: '10px',
      boxShadow: '0 4px 8px rgba(0,0,0,0.2)',
      display: isOpen ? 'flex' : 'none',
      flexDirection: 'column',
      zIndex: 1000,
    },
    header: {
      backgroundColor: '#007bff',
      color: 'white',
      padding: '10px',
      borderTopLeftRadius: '10px',
      borderTopRightRadius: '10px',
      textAlign: 'center',
      fontWeight: 'bold',
    },
    messageContainer: {
      flex: 1,
      padding: '10px',
      overflowY: 'auto',
    },
    message: {
      marginBottom: '10px',
      padding: '8px 12px',
      borderRadius: '18px',
      maxWidth: '80%',
    },
    userMessage: {
      backgroundColor: '#007bff',
      color: 'white',
      alignSelf: 'flex-end',
      marginLeft: 'auto',
    },
    aiMessage: {
      backgroundColor: '#f1f1f1',
      color: 'black',
      alignSelf: 'flex-start',
    },
    inputContainer: {
      display: 'flex',
      padding: '10px',
      borderTop: '1px solid #ccc',
    },
    input: {
      flex: 1,
      border: '1px solid #ccc',
      borderRadius: '20px',
      padding: '8px 12px',
      marginRight: '10px',
    },
    sendButton: {
      backgroundColor: '#007bff',
      color: 'white',
      border: 'none',
      borderRadius: '20px',
      padding: '8px 15px',
      cursor: 'pointer',
    },
  };

  return (
    <>
      <div style={styles.chatButton} onClick={toggleChat}>
        {isOpen ? 'X' : '🔵'}
      </div>
      <div style={styles.chatWindow}>
        <div style={styles.header}>Physical AI Assistant</div>
        <div style={styles.messageContainer}>
          {messages.map((msg, index) => (
            <div
              key={index}
              style={{
                ...styles.message,
                ...(msg.sender === 'user' ? styles.userMessage : styles.aiMessage),
                alignSelf: msg.sender === 'user' ? 'flex-end' : 'flex-start',
                marginLeft: msg.sender === 'user' ? 'auto' : '0',
                marginRight: msg.sender === 'ai' ? 'auto' : '0',
              }}
            >
              {msg.text}
            </div>
          ))}
          {isLoading && <div style={{...styles.message, ...styles.aiMessage}}>...</div>}
        </div>
        <div style={styles.inputContainer}>
          <input
            type="text"
            style={styles.input}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyPress={(e) => e.key === 'Enter' && handleSend()}
            placeholder="Ask a question..."
          />
          <button style={styles.sendButton} onClick={handleSend}>Send</button>
        </div>
      </div>
    </>
  );
};

export default ChatBot;
