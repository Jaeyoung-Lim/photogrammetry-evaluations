dataset?=${CURDIR}/dataset

clean:
	rm -rf dataset/dense/*
	rm -rf dataset/sparse/*
	rm -rf dataset/database.db
	rm -rf output

format:
	Tools/fix_code_style.sh .

increment: SHELL:=/bin/bash
increment:
	./scripts/incremental_reconstruction.sh -d ${dataset} -o ${CURDIR}/output

reconstruct:
	./scripts/reconstruction.sh ${dataset} .

register:
	./scripts/register.sh ${dataset} .
