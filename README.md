# 3DRender

## Setup
1. Eclipse installieren
2. *Existing Project from Workspace* importieren
3. *Java Application* Run Configuration hinzufügen
  1. Main Class `sim.CarbotSim`
  2. Programm Arguments `-e ${workspace_loc:3drender}/nav_environment1.txt -c democontroller.Navigate`
4. Ausführen

## ToDo

- [ ] Texturen für TriangleMesh
- [ ] Lichtquelle positionieren und Schatten entsprechend der Lichtquellenposition nach dem Roth verfahren nutzen
- [x] Reifenspuren zeichnen
- [ ] Himmel und Boden mit Texturen zeichen (evtl. SubScene oder 2D-Grafik)
- [ ] WireFrame-Ansicht (trivial)
- [ ] Optimieren und dem Code eine saubere Struktur versehen
- [ ] Kamera ist noch nicht exakt positioniert (minimaler Unterschied zur bisherigen Ansicht)
