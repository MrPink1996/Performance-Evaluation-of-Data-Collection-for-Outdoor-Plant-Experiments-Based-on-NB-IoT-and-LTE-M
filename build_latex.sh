
echo "Creating the figures..."
python3 create_plots_for_documentation.py

echo "Building the LaTeX document..."
cd documentation
pdflatex main.tex && makeglossaries main
pdflatex main.tex
biber main
pdflatex main.tex
cd ..
