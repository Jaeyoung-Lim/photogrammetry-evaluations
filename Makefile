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
	./scripts/incremental_reconstruction.sh -d ${dataset} -o /mnt/storage/output

reconstruct:
	./scripts/reconstruction.sh ${dataset} .

process-geotag:
	./scripts/process_geotag.sh ${dataset} ${dataset}

register:
	./scripts/register.sh ${dataset} .
