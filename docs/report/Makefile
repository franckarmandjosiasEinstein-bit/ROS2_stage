# =====================================================================
#  Build the PFA technical report.
#
#    make          -> report.pdf   (full: pdflatex, bibtex, pdflatex x2)
#    make quick    -> report.pdf   (one pass; references will be wrong)
#    make clean    -> remove build artefacts, keep the PDF
#    make purge    -> remove everything generated, including the PDF
#    make check    -> report undefined references and citations
#
#  Requires: texlive with IEEEtran, tikz, listings, algorithmicx.
#      sudo apt install texlive-latex-extra texlive-publishers \
#                       texlive-science texlive-fonts-recommended
#  (or simply: sudo apt install texlive-full)
# =====================================================================

MAIN   = report
LATEX  = pdflatex -interaction=nonstopmode -halt-on-error
BIBTEX = bibtex

SRC = $(MAIN).tex preamble.tex references.bib \
      $(wildcard sections/*.tex) $(wildcard figures/*.tex)

.PHONY: all quick clean purge check watch

all: $(MAIN).pdf

$(MAIN).pdf: $(SRC)
	@echo "--> pre-flight (pure Python, no LaTeX needed)"
	@python3 check_latex.py || exit 1
	@echo "--> pass 1/4 (pdflatex)"
	@$(LATEX) $(MAIN) > /dev/null || (tail -40 $(MAIN).log; exit 1)
	@echo "--> pass 2/4 (bibtex)"
	-@$(BIBTEX) $(MAIN) > /dev/null
	@echo "--> pass 3/4 (pdflatex)"
	@$(LATEX) $(MAIN) > /dev/null || (tail -40 $(MAIN).log; exit 1)
	@echo "--> pass 4/4 (pdflatex)"
	@$(LATEX) $(MAIN) > /dev/null || (tail -40 $(MAIN).log; exit 1)
	@echo
	@echo "    $(MAIN).pdf   $$(pdfinfo $(MAIN).pdf 2>/dev/null | \
	        awk '/Pages/{print $$2" pages"}')"
	@$(MAKE) --no-print-directory check

quick:
	@$(LATEX) $(MAIN) || (tail -40 $(MAIN).log; exit 1)

# Surface the two problems that silently degrade an IEEE submission:
# a "??" cross-reference and a "[?]" citation.
check:
	@u=$$(grep -c 'undefined' $(MAIN).log 2>/dev/null || echo 0); \
	 if [ "$$u" -gt 0 ]; then \
	   echo "    WARNING: $$u undefined reference(s)/citation(s):"; \
	   grep -n 'undefined' $(MAIN).log | sed 's/^/      /' | head -20; \
	 else \
	   echo "    all references and citations resolved."; \
	 fi

clean:
	@rm -f $(MAIN).aux $(MAIN).bbl $(MAIN).blg $(MAIN).log $(MAIN).out \
	       $(MAIN).toc $(MAIN).lof $(MAIN).lot $(MAIN).synctex.gz \
	       sections/*.aux figures/*.aux
	@echo "    build artefacts removed ($(MAIN).pdf kept)."

purge: clean
	@rm -f $(MAIN).pdf
	@echo "    $(MAIN).pdf removed."
