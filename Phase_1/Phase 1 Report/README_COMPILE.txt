====================================================================
FYP B28 -- Phase I Review I Report
Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery
SSN College of Engineering, Department of CSE
====================================================================

STEP 1: COPY REQUIRED FILES FROM SENIORS' FOLDER
-------------------------------------------------
Copy these files from:
  C:\Users\thiru\OneDrive - SSN-Institute\SEM 7\FYP - Drone\
  Reference Material (From Seniors - just for format)\FYP_Phase1_Report\

Into this folder (FYP_B28_Review1\):
  - Thesis.cls
  - bmacros.tex
  - lstpatch.sty
  - setspace.sty
  - ssn.jpeg

STEP 2: GENERATE PLANTUML DIAGRAMS
-----------------------------------
Download PlantUML: https://plantuml.com/download
You need Java installed.

Run from this folder:
  java -jar plantuml.jar diagrams\arch.puml
  java -jar plantuml.jar diagrams\tier1.puml
  java -jar plantuml.jar diagrams\tier2.puml
  java -jar plantuml.jar diagrams\gantt.puml

This creates:
  diagrams\arch.png
  diagrams\tier1.png
  diagrams\tier2.png
  diagrams\gantt.png

Alternatively, paste the .puml file contents into:
  https://www.plantuml.com/plantuml/uml/
and download the PNG. Save to the diagrams\ folder.

STEP 3: COMPILE LATEX
---------------------
Recommended: Use TeXstudio or Overleaf.

Command-line (BibTeX must run between the first two pdflatex passes):
  pdflatex main.tex
  bibtex main
  pdflatex main.tex
  pdflatex main.tex

TROUBLESHOOTING
---------------
- If you get "Cannot find Thesis.cls": make sure you copied it from the seniors' folder.
- If you get "File ssn.jpeg not found": copy ssn.jpeg from the seniors' folder.
- If algpseudocode is missing: install algorithmicx package.
- If the glossaries package causes errors, comment out \makeglossaries in Thesis.cls.
- If diagrams/arch.png is missing, LaTeX will warn but compile with a placeholder.

PACKAGE REQUIREMENTS (beyond standard LaTeX)
---------------------------------------------
  algpseudocode  -- Pseudocode (part of algorithmicx)
  algorithm      -- Algorithm float
  booktabs       -- Professional table lines
  multirow       -- Multi-row table cells
  xcolor         -- Colour support
  url            -- URL formatting in references

NOTES ON REFERENCES
--------------------
References are stored in references.bib and compiled via BibTeX
(bibliographystyle: unsrtnat, from the natbib package).
Citation keys used in text: \cite{chokkalingam2026}, \cite{datsko2024},
\cite{ntakolia2023}, \cite{osmnx}, \cite{networkx}, \cite{mopso},
\cite{ardupilot}, \cite{mavlink}

VERIFY paper details via DOI links before submission:
  [1] https://doi.org/10.1051/ro/2025159
  [2] https://ieeexplore.ieee.org/document/10414185
  [3] https://doi.org/10.1007/s10846-023-01945-9

====================================================================
