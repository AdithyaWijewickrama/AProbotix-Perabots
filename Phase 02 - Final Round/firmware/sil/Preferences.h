#pragma once
// file-backed NVS stand-in (sil_prefs.bin next to the binary) so a Section 3 restart works in SIL
#include <stdio.h>
#include <string.h>
#include <map>
#include <string>
#include <vector>
struct Preferences {
  std::map<std::string, std::vector<char>> kv;
  const char *path() { return getenv("SIL_PREFS") ? getenv("SIL_PREFS") : "sil_prefs.bin"; }
  void load() { kv.clear(); FILE *f = fopen(path(), "rb"); if (!f) return; char k[64]; unsigned n;
    while (fscanf(f, "%63s %u\n", k, &n) == 2) { std::vector<char> v(n); if (fread(v.data(), 1, n, f) != n) break; fgetc(f); kv[k] = v; } fclose(f); }
  void save() { FILE *f = fopen(path(), "wb"); if (!f) return; for (auto &e : kv) { fprintf(f, "%s %u\n", e.first.c_str(), (unsigned)e.second.size());
    fwrite(e.second.data(), 1, e.second.size(), f); fputc('\n', f); } fclose(f); }
  bool begin(const char *, bool = false) { load(); return true; }
  void end() { save(); }
  int getInt(const char *k, int d = 0) { auto i = kv.find(k); if (i == kv.end()) return d; int v; memcpy(&v, i->second.data(), 4); return v; }
  void putInt(const char *k, int v) { kv[k] = std::vector<char>((char *)&v, (char *)&v + 4); }
  size_t getBytes(const char *k, void *b, size_t n) { auto i = kv.find(k); if (i == kv.end() || i->second.size() != n) return 0; memcpy(b, i->second.data(), n); return n; }
  size_t putBytes(const char *k, const void *b, size_t n) { kv[k] = std::vector<char>((const char *)b, (const char *)b + n); return n; }
};
