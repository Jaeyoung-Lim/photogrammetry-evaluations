path?=${CURDIR}/dataset

clean:
	rm -rf dataset/dense/*
	rm -rf dataset/sparse/*
	rm -rf dataset/database.db
	rm -rf output

increment:
	./scripts/incremental_reconstruction.sh ${path} ${CURDIR}

reconstruct:
	./scripts/reconstruction.sh ${path} .

register:
	./scripts/register.sh ${path} .
