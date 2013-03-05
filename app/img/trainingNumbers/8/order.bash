number=8
decompte=1

for file in *.png
do
  mv "$file" "$number-$decompte.png"
  ((decompte++))
done
