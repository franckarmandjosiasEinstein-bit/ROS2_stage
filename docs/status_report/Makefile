# Build the 8-page status report.
#   make        -> status.pdf
#   make check  -> pre-compile checks (works without a LaTeX install)
#   make clean

TARGET = status

all: $(TARGET).pdf

$(TARGET).pdf: $(TARGET).tex preamble.tex
	pdflatex -interaction=nonstopmode -halt-on-error $(TARGET).tex
	pdflatex -interaction=nonstopmode -halt-on-error $(TARGET).tex

check:
	python3 ../report/check_latex.py .

pages: $(TARGET).pdf
	@pdfinfo $(TARGET).pdf | grep Pages || true

clean:
	rm -f $(TARGET).aux $(TARGET).log $(TARGET).out $(TARGET).toc

.PHONY: all check pages clean
