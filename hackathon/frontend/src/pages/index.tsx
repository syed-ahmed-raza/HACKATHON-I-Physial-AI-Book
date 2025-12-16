import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

// --- Hero Section (Dark GenAI Look) ---
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header style={{
      background: 'radial-gradient(circle at 50% -20%, #2b1055 0%, #000000 100%)', // Deep Purple to Black
      padding: '10rem 0 6rem 0',
      textAlign: 'center',
      color: 'white',
      position: 'relative',
      overflow: 'hidden'
    }}>
      {/* Background Glow Effect */}
      <div style={{
        position: 'absolute', top: '-100px', left: '50%', transform: 'translateX(-50%)',
        width: '600px', height: '600px', background: '#7c3aed', opacity: '0.15', filter: 'blur(100px)', borderRadius: '50%', zIndex: 0
      }} />

      <div className="container" style={{position: 'relative', zIndex: 1}}>
        <div style={{
             textTransform: 'uppercase', letterSpacing: '2px', fontSize: '0.9rem', fontWeight: 'bold', 
             color: '#a78bfa', marginBottom: '1rem' 
        }}>
          Authored by Syed Ahmed Raza
        </div>

        <Heading as="h1" style={{
            fontSize: '4.5rem', 
            fontWeight: '800', 
            lineHeight: '1.1',
            marginBottom: '1.5rem',
            background: 'linear-gradient(to right, #ffffff, #a5b4fc)',
            WebkitBackgroundClip: 'text',
            WebkitTextFillColor: 'transparent'
        }}>
          Turn Concepts into <br />
          Embodied Intelligence
        </Heading>
        
        <p style={{
            fontSize: '1.25rem', 
            color: '#d1d5db', 
            maxWidth: '700px', 
            margin: '0 auto 2.5rem auto',
            lineHeight: '1.6'
        }}>
          A comprehensive textbook bridging ROS 2, Simulation, and Generative AI. 
          Empower your robotics journey with <span style={{color: '#a78bfa', fontWeight: 'bold'}}>Physical AI</span>.
        </p>
        
        <div style={{display: 'flex', gap: '1rem', justifyContent: 'center'}}>
          <Link
            className="button"
            to="/docs/module-1-ros2/foundations-of-physical-ai"
            style={{
                padding: '1rem 3rem', 
                fontSize: '1.2rem',
                borderRadius: '50px',
                background: 'linear-gradient(90deg, #6366f1 0%, #a855f7 100%)', // Purple/Blue Gradient Button
                border: 'none',
                color: 'white',
                fontWeight: 'bold',
                boxShadow: '0 10px 25px -5px rgba(124, 58, 237, 0.5)',
                transition: 'transform 0.2s'
            }}>
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

// --- Curriculum Cards (Dark Theme) ---
function HomepageCurriculum() {
  const modules = [
    {
      title: 'Module 1: The Nervous System',
      description: 'Focus: ROS 2 & Middleware for robotic communication and control.',
      link: '/docs/module-1-ros2/foundations-of-physical-ai', // Corrected path
    },
    {
      title: 'Module 2: The Digital Twin',
      description: 'Focus: Gazebo & Unity Simulation for virtual robot environments.',
      link: '/docs/module-2-simulation/gazebo-simulation-setup', // Corrected path
    },
    {
      title: 'Module 3: The AI Brain',
      description: 'Focus: NVIDIA Isaac & Reinforcement Learning for advanced robotic intelligence.',
      link: '/docs/module-3-isaac/nvidia-isaac-ecosystem', // Corrected path
    },
    {
      title: 'Module 4: Generative Robotics',
      description: 'Focus: VLA Models & GPT-4o for natural language understanding and action.',
      link: '/docs/module-4-vla/voice-to-action', // Corrected path
    },
  ];

  return (
    <section style={{background: '#050505', padding: '5rem 0'}}>
      <div className="container">
        <h2 style={{
            textAlign: 'center', color: 'white', fontSize: '2.5rem', marginBottom: '3rem', fontWeight: 'bold'
        }}>Curriculum</h2>
        <div className="row">
          {modules.map((module, idx) => (
            <div key={idx} className="col col--6 margin-bottom--lg">
              <div className="card" style={{
                  backgroundColor: '#111113', 
                  border: '1px solid #27272a', 
                  color: 'white',
                  height: '100%'
              }}>
                <div className="card__header">
                  <h3 style={{color: '#a78bfa'}}>{module.title}</h3>
                </div>
                <div className="card__body">
                  <p style={{color: '#a1a1aa'}}>{module.description}</p>
                </div>
                <div className="card__footer">
                  <Link
                    className="button button--secondary button--block"
                    to={module.link}
                    style={{backgroundColor: '#27272a', border: 'none', color: 'white'}}>
                    Explore Module
                  </Link>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

import ChatBot from '../components/ChatBot';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Physical AI & Humanoid Robotics Textbook">
      <HomepageHeader />
      <main>
        <HomepageCurriculum />
      </main>
      <ChatBot />
    </Layout>
  );
}