path?=/home/jaeyoung/Downloads/gerrard-hall

clean:
	rm -rf output

increment:
	./scripts/incremental_reconstruction.sh ${path} .

reconstruct:
	./scripts/reconstruction.sh ${path} .
