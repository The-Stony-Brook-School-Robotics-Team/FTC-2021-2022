import React from 'react';
import PropTypes from 'prop-types';

const TileGrid = ({ gridTemplate, children }) => (
  <div className="tile-grid" style={{ gridTemplate, backgroundColor: "rgba(38, 38, 38)" }}>
    {children}
  </div>
);

TileGrid.propTypes = {
  gridTemplate: PropTypes.string.isRequired,
  children: PropTypes.node.isRequired,
};

export default TileGrid;
