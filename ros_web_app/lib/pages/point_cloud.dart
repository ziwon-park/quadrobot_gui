import 'package:flutter/material.dart';
import 'dart:html' as html;
import 'dart:ui' as ui;
import 'dart:convert';

class PointCloudViewer extends StatefulWidget {
  final List<List<double>> points;
  final String title;

  const PointCloudViewer({
    Key? key,
    required this.points,
    this.title = 'Point Cloud Visualization',
  }) : super(key: key);

  @override
  State<PointCloudViewer> createState() => _PointCloudViewerState();
}

class _PointCloudViewerState extends State<PointCloudViewer> {
  late final String viewId;

  @override
  void initState() {
    super.initState();
    viewId = 'point-cloud-view-${DateTime.now().millisecondsSinceEpoch}';
    ui.platformViewRegistry.registerViewFactory(
      viewId, 
      (int viewId) => _createHtmlElement(),
    );
  }

  html.HtmlElement _createHtmlElement() {
    final div = html.DivElement()
      ..style.width = '100%'
      ..style.height = '100%'
      ..id = 'plot';
    
    final pointsJson = jsonEncode(widget.points);
    
    // Plotly.js 스크립트 추가
    final plotlyScript = html.ScriptElement()
      ..src = 'https://cdn.plot.ly/plotly-latest.min.js';
    div.children.add(plotlyScript);
    
    // 데이터 시각화 스크립트 추가
    final visualizationScript = html.ScriptElement()
      ..text = '''
        const points = $pointsJson;
        
        const trace = {
          type: 'scatter3d',
          mode: 'markers',
          x: points.map(p => p[0]),
          y: points.map(p => p[1]),
          z: points.map(p => p[2]),
          marker: {
            size: 2,
            opacity: 0.8,
          }
        };

        const layout = {
          margin: { l: 0, r: 0, b: 0, t: 0 },
          scene: {
            camera: {
              eye: { x: 1.5, y: 1.5, z: 1.5 }
            }
          }
        };

        plotlyScript.onload = () => {
          Plotly.newPlot('plot', [trace], layout);
        };
      ''';
    div.children.add(visualizationScript);

    return div;
  }

  @override
  Widget build(BuildContext context) {
    return HtmlElementView(viewType: viewId);
  }
}