import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

import { connectControl } from '@/services/control-socket';

import { App } from './App';

import './index.css';

connectControl();

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
