
run:
	mkdir output -p
	./paper_run.sh output

report.pdf : 1080_violins.pdf tegra_violins.pdf
	pdfunite 1080_violins.pdf tegra_violins.pdf  $@

%_violins.pdf  : % ./*.py 
	./violins.py $*/violons_* --plot $*_violins.pdf
	pdflatex $*_report.tex

clean :
	rm *.aux *.pdf *.tex *.log -f
