path?=${CURDIR}/dataset

clean:
	rm -rf dataset/*
	

increment:
	./scripts/incremental_reconstruction.sh ${path} .

reconstruct:
	./scripts/reconstruction.sh ${path} .

register:
	./scripts/register.sh ${path} .
