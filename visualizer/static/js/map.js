// Initialize map with center coordinates and appropriate zoom level
var map = L.map('map').setView([39.941326, -75.199492], 16);

// Try different tile providers - one of these should work
// 1. OpenStreetMap Standard
var mapboxDetailed = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
    attribution: '© <a href="https://www.mapbox.com/about/maps/">Mapbox</a>',
    maxZoom: 30,
    id: 'mapbox/satellite-v9',
    accessToken: 'pk.eyJ1IjoiamFzb25haCIsImEiOiJjbThxOWt5ZnMwa3NxMmtwdTEwYjJ1ajZ3In0.LGKzoIXaNi9j7lXhypfBPQ'
}).addTo(map);



var baseLayers = {
    "MapBox": mapboxDetailed
};

L.control.layers(baseLayers).addTo(map);

// Add a console log to check if the map initialized
console.log("Map initialized with center:", map.getCenter(), "zoom:", map.getZoom());

// Create layers for your data
var pointsLayer = L.layerGroup().addTo(map);
var pathLayer = L.layerGroup().addTo(map);
var objectsLayer = L.layerGroup().addTo(map);
var connectionsLayer = L.layerGroup().addTo(map);

// Rest of your code remains the same...
// Create layers
var pointsLayer = L.layerGroup().addTo(map);
var pathLayer = L.layerGroup().addTo(map);
var objectsLayer = L.layerGroup().addTo(map);
var connectionsLayer = L.layerGroup().addTo(map);

// Create a legend
var legend = L.control({position: 'bottomright'});

legend.onAdd = function (map) {
    var div = L.DomUtil.create('div', 'info legend');
    div.style.backgroundColor = 'rgba(255, 255, 255, 0.8)';
    div.style.padding = '10px';
    div.style.borderRadius = '5px';
    div.style.boxShadow = '0 0 15px rgba(0,0,0,0.2)';
    
    // Add title
    div.innerHTML = '<h4 style="margin-top: 0; margin-bottom: 10px; text-align: center;">Legend</h4>';
    
    // Define your legend items here
    // Format: [color, label]
    var items = [
        ['#FF3333', 'Jackal Position'],
        // Add your object types here with their colors
        // For example:
        //['#28B463', 'Objects'],
        //['#6495ED', 'Roads'],
    ];
    
    // Collect unique colors from objects to populate legend dynamically
    var objectColors = {};
    
    // Function to update legend with object colors
    window.updateLegendWithObjectColors = function(objects) {
        if (!objects || objects.length === 0) return;
        
        objects.forEach(function(obj) {
            if (obj.color && obj.label) {
                objectColors[obj.color] = obj.label;
            }
        });
        
        // Rebuild items array
        items = [['#FF0000', 'Current Position/Track']];
        
        // Add object colors to items
        for (var color in objectColors) {
            items.push([color, objectColors[color]]);
        }
        
        // Update legend HTML
        updateLegendHTML();
    };
    
    // Function to update legend HTML
    function updateLegendHTML() {
        var legendHTML = '<h4 style="margin-top: 0; margin-bottom: 10px; text-align: center;">Legend</h4>';
        
        // Add each legend item
        for (var i = 0; i < items.length; i++) {
            legendHTML += 
                '<div style="display: flex; align-items: center; margin-bottom: 5px;">' +
                    '<span style="background:' + items[i][0] + '; width: 15px; height: 15px; border-radius: 50%; display: inline-block; margin-right: 5px; border: 1px solid #FFF;"></span> ' +
                    '<span>' + items[i][1] + '</span>' +
                '</div>';
        }
        
        div.innerHTML = legendHTML;
    }
    
    // Initial legend HTML update
    updateLegendHTML();
    
    return div;
};

// Add legend to map
legend.addTo(map);


// Connect to WebSocket
var socket = io();

// Listen for GPS updates
socket.on('gps_update', function(data) {
    console.log("Received GPS update:", data);
    
    // Clear previous markers and paths
    pointsLayer.clearLayers();
    pathLayer.clearLayers();
    
    if (data.points.length === 0) return;
    
    // Set the color to use for GPS track
    var trackColor = '#FF3333';  // Red color
    
    // Extract all coordinates for the path
    var pathCoords = data.points.map(point => [point.lat, point.lon]);
    
    // Draw the connecting line for all points
    if (pathCoords.length > 1) {
        L.polyline(pathCoords, {
            color: trackColor,
            weight: 3,
            opacity: 0.7
        }).addTo(pathLayer);
    }
    
    // Add only the most recent point as a red circle marker
    var lastPoint = data.points[data.points.length - 1];
    
    // Use circleMarker instead of standard marker
    L.circleMarker([lastPoint.lat, lastPoint.lon], {
        radius: 8,
        fillColor: trackColor,
        color: '#FFFFFF',  // White border
        weight: 2,
        opacity: 1,
        fillOpacity: 1
    })
    .bindPopup(lastPoint.popup || "Current Position")
    .addTo(pointsLayer);
});
