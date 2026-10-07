#ifndef SV6X66_IDENTITY_H
#define SV6X66_IDENTITY_H

/* Initialize both radio identities from read-only efuse APIs before WIFI_INIT.
 * Returns 1 on success, 0 if no valid pair is available. Success is cached;
 * a failed attempt leaves the cache untouched and may be retried. */
int SV6X66_IdentityInit(void);

#endif
