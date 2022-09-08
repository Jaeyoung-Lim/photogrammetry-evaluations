path?=${CURDIR}/dataset

clean:
	rm -rf dataset/dense/*
	rm -rf dataset/sparse/*
	rm -rf dataset/database.db
	rm -rf output

increment: SHELL:=/bin/bash
increment:
	./scripts/incremental_reconstruction.sh -d ${path} -o ${CURDIR}/output

reconstruct:
	./scripts/reconstruction.sh ${path} .

register:
	./scripts/register.sh ${path} .
