import fitz
import os

doc = fitz.open(r'D:\PeraBots\Final Round\UNIVERSITY CATEGORY FINAL ROUND RULEBOOK.pdf')
out_dir = r'D:\PeraBots\Final Round\pdf_images'
os.makedirs(out_dir, exist_ok=True)

for page_num in range(doc.page_count):
    page = doc[page_num]
    images = page.get_images(full=True)
    for img_idx, img in enumerate(images):
        xref = img[0]
        try:
            base_image = doc.extract_image(xref)
            image_bytes = base_image['image']
            ext = base_image['ext']
            w = base_image['width']
            h = base_image['height']
            fname = "page{}_img{}.{}".format(page_num+1, img_idx+1, ext)
            with open(os.path.join(out_dir, fname), 'wb') as f:
                f.write(image_bytes)
            print("Saved: {} ({} bytes, {}x{})".format(fname, len(image_bytes), w, h))
        except Exception as e:
            print("Error page {} img {}: {}".format(page_num+1, img_idx+1, e))
