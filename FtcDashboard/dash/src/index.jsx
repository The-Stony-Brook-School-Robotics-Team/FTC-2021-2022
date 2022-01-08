import React from 'react';
import ReactDOM from 'react-dom';
import { Provider } from 'react-redux';
import Dashboard from './containers/Dashboard';
import configureStore from './store/configureStore';
import './index.css';

const store = configureStore();

ReactDOM.render(
  <Provider store={store} style={{backgroundColor: "rgba(38, 38, 38)", color: "rgb(254, 254, 254)"}}>
    <Dashboard />
  </Provider>,
  document.getElementById('root'),
);
