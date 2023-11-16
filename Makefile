path?=${CURDIR}/dataset

clean:
	rm -rf dataset/dense/*
	rm -rf dataset/sparse/*
	rm -rf dataset/database.db
	rm -rf output

format:
	Tools/fix_code_style.sh .

increment: SHELL:=/bin/bash
increment:
	./scripts/incremental_reconstruction.sh -d ${path} -o ${CURDIR}/output

reconstruct:
	./scripts/reconstruction.sh ${path} ${CURDIR}/output

register:
	./scripts/register.sh ${path} .
