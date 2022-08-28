path?=${CURDIR}/dataset

clean:
	rm -rf output

increment:
	./scripts/incremental_reconstruction.sh ${path} .

reconstruct:
	./scripts/reconstruction.sh ${path} .
