number=6
decompte=1

for file in *.png
do
  mv "$file" "$number-$decompte.png"
  ((decompte++))
done
