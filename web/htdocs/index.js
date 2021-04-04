var socket = io();

var location_map = null;
var location_map_marker = null;

var spectrum_graph = null;
var spectrum_graph_data = [];

socket.on('connect', function ()
{
    socket.on('update', function (data)
    {
        //console.log(data);

        // 0: Timestamp
        $("#gnss-timestamp").text((new Date(data['0'] * 1000)).toLocaleString());

        // 1: Location
        $("#gnss-latitude").text(roundTo((data['1'][0] / 1.0e7), 4));
        $("#gnss-longitude").text(roundTo((data['1'][1] / 1.0e7), 4));
        $("#gnss-altitude").text(roundTo((data['1'][2] / 1.0e3), 1));

        if(location_map == null)
        {
            location_map = L.map('gnss-location-map').setView([(data['1'][0] / 1.0e7), (data['1'][1] / 1.0e7)], 16);

            location_map_marker = L.marker([(data['1'][0] / 1.0e7), (data['1'][1] / 1.0e7)]).addTo(location_map);

            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
                {
                maxZoom: 19,
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }).addTo(location_map);

            L.control.scale().addTo(location_map);
        }
        else
        {
            oldLatLng = location_map_marker.getLatLng();
            location_map_marker.setLatLng([(data['1'][0] / 1.0e7), (data['1'][1] / 1.0e7)]);
            new L.Polyline([oldLatLng, new L.LatLng((data['1'][0] / 1.0e7), (data['1'][1] / 1.0e7))], {
        color: 'red',
        weight: 3,
        opacity: 0.5,
        smoothFactor: 0
    }).addTo(location_map);
            location_map.setView([(data['1'][0] / 1.0e7), (data['1'][1] / 1.0e7)]);
        }

        // 2: Accuracy
        $("#gnss-hacc").text(roundTo((data['2'][0] / 1.0e3), 1));
        $("#gnss-vacc").text(roundTo((data['2'][1] / 1.0e3), 1));

        // 3: SVs
        $("#svs-acquired").text(data['3'][0]);
        $("#svs-locked").text(data['3'][1]);
        $("#svs-nav").text(data['3'][2]);

        // 4: RF
        $("#gnss-rf-agc").text(data['4'][0]);
        $("#gnss-rf-noise").text(data['4'][1]);

        // 5: Jamming
        $("#gnss-jamming-cw").text(data['5'][0]);
        $("#gnss-jamming-broadband").text(data['5'][1]);
        if(data['5'][1] == 1)
        {
            $("#gnss-jamming-broadband-description").text("OK");
        }
        else if(data['5'][1] == 2)
        {
            $("#gnss-jamming-broadband-description").text("WARNING");
        }
        else if(data['5'][1] == 3)
        {
            $("#gnss-jamming-broadband-description").text("CRITICAL");
        }

        // 10: Spectrum
        var gnss_spectrum_centerfreq = data['10'][0] / 1.0e6;
        var gnss_spectrum_resolution = data['10'][1] / 1.0e6;
        var gnss_spectrum_data = data['10'][2];
        var gnss_spectrum_pgagain = data['10'][3];

        var data_x_start = gnss_spectrum_centerfreq - (128 * gnss_spectrum_resolution);
        var data_x_stop = gnss_spectrum_centerfreq + (128 * gnss_spectrum_resolution);
        spectrum_graph_data = [];
        for(var i = 0; i < gnss_spectrum_data.length; i++)
        {
        spectrum_graph_data.push([data_x_start + (i * gnss_spectrum_resolution), gnss_spectrum_data[i]])
        }
        for(var key in gnss_spectrum_data)
        {
            if(gnss_spectrum_data.hasOwnProperty(key))
            {
                spectrum_graph_data.push([data_x_start + (key * gnss_spectrum_resolution), gnss_spectrum_data[key]])
            }
        }

        if(spectrum_graph == null)
        {
            spectrum_graph = new Dygraph(
                document.getElementById("gnss-spectrum-graph"),
                spectrum_graph_data,
                {
                    valueRange: [0.0, 255.0],
                    labels: ['Frequency', 'Power'],
                    fillGraph: true,
                    axes: {
                        y: {
                            drawAxis: false
                        }
                    }
                }
            );
        }
        else
        {
            spectrum_graph.updateOptions( { 'file': spectrum_graph_data } );
        }

    });
});

var roundTo = function(n, d) {
  'use strict';
  var r, e;
  d = Math.pow(10, d);
  e = n * d;
  r = Math.round(e);
  return r / d;
};
