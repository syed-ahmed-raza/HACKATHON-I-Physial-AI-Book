import React, { useState, CSSProperties } from 'react';

// Message type define karna zaroori hai TS mein
interface Message {
  sender: 'user' | 'ai';
  text: string;
}

const ChatBot: React.FC = () => {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState<string>('');
  const [isLoading, setIsLoading] = useState<boolean>(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && messages.length === 0) {
      setMessages([{ sender: 'ai', text: 'Hello! How can I help you with the Physical AI textbook today?' }]);
    }
  };

  const handleSend = async () => {
    if (!input.trim()) return;

    const userMessage: Message = { sender: 'user', text: input };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Render URL
      const response = await fetch('https://physical-ai-backend-kxci.onrender.com/chat', {
        method: 'POST',
        body: JSON.stringify({ query: input }),
        headers: { 'Content-Type': 'application/json' },
      });

      if (!response.ok) {
        throw new Error('Server not reachable');
      }

      const data = await response.json();
      const aiMessage: Message = { sender: 'ai', text: data.response };
      setMessages((prev) => [...prev, aiMessage]);
    } catch (error) {
      const errorMessage: Message = { sender: 'ai', text: 'Error: Could not connect to the server.' };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // ✅ Styles ko 'CSSProperties' type de di taake TS error na de
  const styles: { [key: string]: CSSProperties } = {
    chatButton: {
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      width: '60px',
      height: '60px',
      borderRadius: '50%',
      backgroundColor: '#250069ff',
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
      flexDirection: 'column', // Ab ye error nahi dega
      zIndex: 1000,
    },
    header: {
      backgroundColor: '#250069ff',
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
      backgroundColor: '#250069ff',
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
      backgroundColor: '#250069ff',
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
            style={styles.input as CSSProperties} // Small cast ensures safety
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