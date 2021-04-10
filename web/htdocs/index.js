'use strict';

const highlight_colour_beidou = "rgba(255, 193, 204, 1.0)";
const highlight_colour_galileo = "rgba(178, 241, 255, 1.0)";
const highlight_colour_gps = "rgba(163, 255, 153, 1.0)";
const highlight_colour_glonass = "rgba(241, 222, 197, 1.0)";

var socket = io();

var location_map = null;
var location_map_marker = null;

var spectrum_graph = null;
var spectrum_graph_data = [];

var spectrum_graph2 = null;
var spectrum_graph2_data = [];

socket.on('connect', function ()
{
    socket.on('update', function (data)
    {
        //console.log(data);

        var multiband = false;
        if('11' in data)
        {
            multiband = true;
            $(".l2_elements").show();
        }

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
            const oldLatLng = location_map_marker.getLatLng();
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
        if(!multiband)
        {
            $("#svs-acquired").text(data['3'][0]);
            $("#svs-locked").text(data['3'][2]);
        }
        else
        {
            $("#svs-acquired").text(`${data['3'][0]}·${data['3'][1]}`);
            $("#svs-locked").text(`${data['3'][2]}·${data['3'][3]}`);
        }
        $("#svs-nav").text(data['3'][4]);

        // 4: RF
        if(!multiband)
        {
            $("#gnss-rf-agc").text(data['4'][0]);
            $("#gnss-rf-noise").text(data['4'][1]);
        }
        else
        {
            $("#gnss-rf-agc").text(`${data['4'][0]}·${data['6'][0]}`);
            $("#gnss-rf-noise").text(`${data['4'][1]}·${data['6'][1]}`);
        }

        // 5: Jamming
        $("#gnss-jamming-cw").text(data['5'][0]);
        $("#gnss-jamming-broadband").text(data['5'][1]);
        if(data['5'][1] == 0)
        {
            $("#gnss-jamming-broadband-description").text("UNKNOWN");
        }
        else if(data['5'][1] == 1)
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
        if(multiband)
        {
            $("#gnss2-jamming-cw").text(data['7'][0]);
            $("#gnss2-jamming-broadband").text(data['7'][1]);
            if(data['7'][1] == 0)
            {
                $("#gnss2-jamming-broadband-description").text("UNKNOWN");
            }
            else if(data['7'][1] == 1)
            {
                $("#gnss2-jamming-broadband-description").text("OK");
            }
            else if(data['7'][1] == 2)
            {
                $("#gnss2-jamming-broadband-description").text("WARNING");
            }
            else if(data['7'][1] == 3)
            {
                $("#gnss2-jamming-broadband-description").text("CRITICAL");
            }
        }

        // 10: Spectrum
        var gnss_spectrum_centerfreq = data['10'][0] / 1.0e6;
        var gnss_spectrum_resolution = data['10'][1] / 1.0e6;
        var gnss_spectrum_data = new Uint8Array(data['10'][2]);
        var gnss_spectrum_pgagain = data['10'][3];

        var data_x_start = gnss_spectrum_centerfreq - (128 * gnss_spectrum_resolution);
        var data_x_stop = gnss_spectrum_centerfreq + (128 * gnss_spectrum_resolution);
        spectrum_graph_data = [];
        gnss_spectrum_data.forEach((element, index) => {
            spectrum_graph_data.push([roundTo(data_x_start + (index * gnss_spectrum_resolution),1), element])
        });

        if(spectrum_graph == null)
        {
            spectrum_graph = new Dygraph(
                document.getElementById("gnss-spectrum-graph"),
                spectrum_graph_data,
                {
                    valueRange: [0.0, 255.0],
                    title: 'L1 / E1 / B1',
                    titleHeight: 24,
                    labels: ['Frequency', 'Power'],
                    xlabel: 'Frequency (MHz)',
                    fillGraph: true,
                    axes: {
                        y: {
                            drawAxis: false
                        },
                        x: {
                            ticker: function(min, max, pixels) {
                                return [
                                    { v: 1525 },
                                    { label_v: 1525, label: '1525' },
                                    { v: 1550 },
                                    { label_v: 1550, label: '1550' },
                                    { v: 1575 },
                                    { label_v: 1575, label: '1575' },
                                    { v: 1600 },
                                    { label_v: 1600, label: '1600' },
                                    { v: 1625 },
                                    { label_v: 1625, label: '1625' },
                                ]
                           }
                        }
                    },
                    underlayCallback: function(canvas, area, g) {
                        /* BEIDOU B1 */
                        highlightGraph(1559, 1592, 10, highlight_colour_beidou, canvas, area, g);
                        labelGraph(1575.42, 25, "B1", canvas, area, g);
                        /* Galileo E1 */
                        highlightGraph(1563, 1588, 30, highlight_colour_galileo, canvas, area, g);
                        labelGraph(1575.42, 45, "E1", canvas, area, g);
                        /* GPS L1 */
                        highlightGraph(1565, 1586, 50, highlight_colour_gps, canvas, area, g);
                        labelGraph(1575.42, 65, "L1", canvas, area, g);
                        /* GLONASS L1 */
                        highlightGraph(1596, 1606, 10, highlight_colour_glonass, canvas, area, g);

                        labelGraph(1601, 30, "G1", canvas, area, g);
                    }
                }
            );
        }
        else
        {
            spectrum_graph.updateOptions( { 'file': spectrum_graph_data } );
        }

        // 11: L2 Spectrum (optional)
        if(multiband)
        {
            var gnss_spectrum_centerfreq = data['11'][0] / 1.0e6;
            var gnss_spectrum_resolution = data['11'][1] / 1.0e6;
            var gnss_spectrum_data = new Uint8Array(data['11'][2]);
            var gnss_spectrum_pgagain = data['11'][3];

            var data_x_start = gnss_spectrum_centerfreq - (128 * gnss_spectrum_resolution);
            var data_x_stop = gnss_spectrum_centerfreq + (128 * gnss_spectrum_resolution);
            spectrum_graph2_data = [];
            gnss_spectrum_data.forEach((element, index) => {
                spectrum_graph2_data.push([roundTo(data_x_start + (index * gnss_spectrum_resolution),1), element])
            });

            if(spectrum_graph2 == null)
            {
                spectrum_graph2 = new Dygraph(
                    document.getElementById("gnss-spectrum2-graph"),
                    spectrum_graph2_data,
                    {
                        valueRange: [0.0, 255.0],
                        title: 'L2 / E5 / B2',
                        titleHeight: 24,
                        labels: ['Frequency', 'Power'],
                        xlabel: 'Frequency (MHz)',
                        fillGraph: true,
                        axes: {
                            y: {
                                drawAxis: false
                            },
                            x: {
                                ticker: function(min, max, pixels) {
                                    return [
                                        { v: 1175 },
                                        { label_v: 1175, label: '1175' },
                                        { v: 1200 },
                                        { label_v: 1200, label: '1200' },
                                        { v: 1225 },
                                        { label_v: 1225, label: '1225' },
                                        { v: 1250 },
                                        { label_v: 1250, label: '1250' },
                                        { v: 1275 },
                                        { label_v: 1275, label: '1275' },
                                    ]
                               }
                            }
                        },
                        underlayCallback: function(canvas, area, g) {
                            /* BEIDOU B2 */
                            highlightGraph(1195.14, 1219.14, 10, highlight_colour_beidou, canvas, area, g);
                            labelGraph(1207.14, 25, "B2", canvas, area, g);
                            /* Galileo E5b */
                            highlightGraph(1196.91, 1217.37, 30, highlight_colour_galileo, canvas, area, g);
                            labelGraph(1207.2, 45, "E5b", canvas, area, g);
                            /* GPS L2 */
                            highlightGraph(1217, 1238, 50, highlight_colour_gps, canvas, area, g);
                            labelGraph(1227.5, 65, "L2", canvas, area, g);
                            /* GLONASS L2 */
                            highlightGraph(1241, 1255, 10, highlight_colour_glonass, canvas, area, g);
                            labelGraph(1248, 25, "G2", canvas, area, g);
                        }
                    }
                );
            }
            else
            {
                spectrum_graph2.updateOptions( { 'file': spectrum_graph2_data } );
            }
        }

    });
});

var roundTo = function(n, d)
{
  var r, e;
  d = Math.pow(10, d);
  e = n * d;
  r = Math.round(e);
  return r / d;
}

var highlightGraph = function(f1, f2, yoffset, colourString, canvas, area, g)
{
    const bottom_left = g.toDomCoords(f1, -20);
    const top_right = g.toDomCoords(f2, +20);

    const left = bottom_left[0];
    const right = top_right[0];

    canvas.fillStyle = colourString;
    canvas.fillRect(left, area.y+yoffset, right - left, area.h);
}

var labelGraph = function(xf, yoffset, textString, canvas, area, g)
{
    const xoffset = g.toDomCoords(xf, 0);

    canvas.fillStyle = "rgba(0, 0, 0, 1.0)";
    canvas.font = "12px Arial";
    canvas.textAlign = "center";
    canvas.fillText(textString, xoffset[0], area.y+yoffset);
}