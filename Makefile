path?=/home/jaeyoung/Downloads/gerrard-hall

clean:
	rm -rf output

run:
	./generate_subset.sh ${path} .
